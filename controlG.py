import tkinter as tk
from tkinter import ttk, messagebox
import math
from pymycobot import MyCobot320

PORT = '/dev/ttyAMA0'
BAUD = 115200

try:
    mc = MyCobot320(PORT, BAUD)
    mc.power_on()
except Exception as e:
    print(f"Error de conexión: {e}")
    mc = None

routine_points = []

# --- Funciones de Validación Matemática ---

def is_safe_cartesian(x, y, z):
    """
    Calcula los límites espaciales permitidos para evitar colisiones.
    """
    Z_MIN = 40  
    if z < Z_MIN:
        return False, f"Violación de límite: Z ({z} mm) es menor al mínimo de seguridad ({Z_MIN} mm)."

    RADIO_MAX = 315  
    radio_actual = math.sqrt(x**2 + y**2)
    if radio_actual > RADIO_MAX:
        return False, f"Fuera de rango: El radio ({radio_actual:.1f} mm) excede la extensión máxima permitida."

    RADIO_MIN_EXCLUSION = 95  
    ALTURA_CRITICA = 160      
    if radio_actual < RADIO_MIN_EXCLUSION and z < ALTURA_CRITICA:
         return False, "Riesgo de colisión: El efector está invadiendo el cilindro central de la base."

    return True, "Posición segura."

def is_safe_angles(j2, j3):
    """
    Reglas heurísticas para cinemática directa.
    """
    if j2 < -45 and j3 > 90:
        return False, "Geometría riesgosa: Inclinación del hombro y pliegue del codo provocan auto-colisión."
    return True, "Ángulos seguros."


# --- Funciones de Control ---

def update_angles(val=None):
    if mc:
        angles = [s.get() for s in angle_sliders]
        seguro, mensaje = is_safe_angles(angles[1], angles[2])
        if seguro:
            mc.send_angles(angles, speed_var.get())
        else:
            print(f"BLOQUEO PREVENTIVO: {mensaje}")

def send_current_coords():
    """Se ejecuta al presionar el botón de Mover a Coordenadas (Sliders)"""
    if mc:
        coords = [s.get() for s in coord_sliders]
        x, y, z = coords[0], coords[1], coords[2]
        
        seguro, mensaje = is_safe_cartesian(x, y, z)
        
        if seguro:
            mc.send_coords(coords, speed_var.get(), 1)
        else:
            messagebox.showwarning("Límite de Seguridad Activado", mensaje)

def send_text_coords():
    """Se ejecuta al presionar el botón de Mover en la nueva pestaña de Texto"""
    if mc:
        try:
            # Extraer valores de los campos de texto
            coords = [float(var.get()) for var in text_coord_vars]
            x, y, z = coords[0], coords[1], coords[2]
            
            # Reutilizamos tu validación matemática para mayor seguridad
            seguro, mensaje = is_safe_cartesian(x, y, z)
            
            if seguro:
                # Ejecutamos mc.send_coords con modo 1 (movimiento lineal)
                mc.send_coords(coords, speed_var.get(), 1)
            else:
                messagebox.showwarning("Límite de Seguridad Activado", mensaje)
        except ValueError:
            messagebox.showerror("Error de Formato", "Por favor ingresa únicamente valores numéricos válidos.")

def go_home():
    if mc:
        mc.send_angles([0, 0, 0, 0, 0, 0], speed_var.get())
        for slider in angle_sliders:
            slider.set(0)

def release_motors():
    if mc:
        mc.release_all_servos()

# --- Funciones de Rutina ---

def save_point():
    if mc:
        current_angles = mc.get_angles()
        if current_angles:
            routine_points.append(current_angles)
            routine_listbox.insert(tk.END, f"P {len(routine_points)}: {current_angles}")

def play_routine(index=0):
    if mc and index < len(routine_points):
        mc.send_angles(routine_points[index], speed_var.get())
        root.after(2500, play_routine, index + 1)

def clear_routine():
    routine_points.clear()
    routine_listbox.delete(0, tk.END)

# --- Función de Monitoreo en Tiempo Real ---

def update_realtime_display():
    """Consulta las coordenadas del robot cada 500ms para actualizar la interfaz"""
    if mc:
        try:
            coords = mc.get_coords()
            if coords and len(coords) == 6:
                for i, val in enumerate(coords):
                    realtime_labels[i].config(text=f"{coord_labels[i]}: {val:.2f}")
        except Exception:
            # Ignorar fallos temporales de lectura del puerto serial
            pass
    # Volver a ejecutar esta función en 500 ms
    root.after(500, update_realtime_display)

# --- Interfaz Gráfica ---

root = tk.Tk()
root.title("MyCobot 320 Pi - Control Seguro")
root.geometry("550x700")

# Panel Global
global_frame = tk.Frame(root)
global_frame.pack(fill='x', pady=10, padx=20)

tk.Label(global_frame, text="Velocidad:", font=("Arial", 10, "bold")).pack(side='left')
speed_var = tk.IntVar(value=40)
speed_slider = tk.Scale(global_frame, from_=1, to=100, orient='horizontal', variable=speed_var, length=150)
speed_slider.pack(side='left', padx=10)

tk.Button(global_frame, text="Ir al Origen", bg="lightblue", command=go_home).pack(side='right')

# Pestañas
notebook = ttk.Notebook(root)
notebook.pack(pady=10, expand=True, fill='both')

# Pestaña 1: Articulaciones
tab_angles = ttk.Frame(notebook)
notebook.add(tab_angles, text="Articulaciones")
angle_limits = [(-168, 168), (-135, 135), (-145, 145), (-148, 148), (-168, 168), (-180, 180)]
angle_sliders = []
for i, limits in enumerate(angle_limits):
    frame = tk.Frame(tab_angles)
    frame.pack(fill='x', padx=10, pady=5)
    tk.Label(frame, text=f"J{i+1}:", width=5).pack(side='left')
    slider = tk.Scale(frame, from_=limits[0], to=limits[1], orient='horizontal', length=400, command=update_angles)
    slider.set(0)
    slider.pack(side='right')
    angle_sliders.append(slider)

# Pestaña 2: Coordenadas (Sliders)
tab_coords = ttk.Frame(notebook)
notebook.add(tab_coords, text="Coordenadas (IK)")
coord_limits = [(-350, 350), (-350, 350), (-41, 523.9), (-180, 180), (-180, 180), (-180, 180)]
coord_labels = ["X", "Y", "Z", "RX", "RY", "RZ"]
coord_sliders = []

tk.Label(tab_coords, text="Ajusta los valores y presiona 'Mover' para validar límites", fg="gray").pack(pady=5)

for i, limits in enumerate(coord_limits):
    frame = tk.Frame(tab_coords)
    frame.pack(fill='x', padx=10, pady=5)
    tk.Label(frame, text=f"{coord_labels[i]}:", width=5).pack(side='left')
    slider = tk.Scale(frame, from_=limits[0], to=limits[1], orient='horizontal', length=400)
    slider.set(200 if coord_labels[i] == 'Z' else 0)
    slider.pack(side='right')
    coord_sliders.append(slider)

tk.Button(tab_coords, text="Mover a Coordenadas", bg="orange", font=("Arial", 11, "bold"), command=send_current_coords).pack(pady=15)

# --- NUEVA PESTAÑA: Coordenadas por Texto y Tiempo Real ---
tab_text_coords = ttk.Frame(notebook)
notebook.add(tab_text_coords, text="Coord (Texto)")

# Contenedor para dividir en dos columnas
columns_frame = tk.Frame(tab_text_coords)
columns_frame.pack(fill='both', expand=True, padx=10, pady=10)

# Columna Izquierda: Monitor en Tiempo Real
left_col = tk.Frame(columns_frame)
left_col.pack(side='left', fill='both', expand=True)

tk.Label(left_col, text="Posición Actual\n(Tiempo Real)", font=("Arial", 10, "bold")).pack(pady=5)
realtime_labels = []
for label_text in coord_labels:
    lbl = tk.Label(left_col, text=f"{label_text}: 0.00", font=("Arial", 11))
    lbl.pack(anchor='w', pady=8, padx=20)
    realtime_labels.append(lbl)

# Columna Derecha: Entradas de Texto
right_col = tk.Frame(columns_frame)
right_col.pack(side='right', fill='both', expand=True)

tk.Label(right_col, text="Modificar Coordenadas\n(Ingresar texto)", font=("Arial", 10, "bold")).pack(pady=5)
text_coord_vars = []
for i, label_text in enumerate(coord_labels):
    f = tk.Frame(right_col)
    f.pack(fill='x', pady=5, padx=10)
    tk.Label(f, text=f"{label_text}:", width=4).pack(side='left')
    
    # Valores por defecto seguros (Z=200)
    var = tk.StringVar(value="200" if label_text == 'Z' else "0")
    entry = tk.Entry(f, textvariable=var, width=12, justify='center')
    entry.pack(side='right')
    text_coord_vars.append(var)

tk.Button(right_col, text="Aplicar Movimiento", bg="orange", font=("Arial", 11, "bold"), command=send_text_coords).pack(pady=20)
# -----------------------------------------------------------

# Pestaña 3: Rutinas
tab_routine = ttk.Frame(notebook)
notebook.add(tab_routine, text="Rutinas")
tk.Label(tab_routine, text="1. Libera los motores.\n2. Mueve el brazo manualmente.\n3. Guarda los puntos.", justify="left").pack(pady=10)

btn_frame = tk.Frame(tab_routine)
btn_frame.pack(fill='x', padx=20)
tk.Button(btn_frame, text="Guardar Punto", bg="lightgreen", command=save_point).pack(side='left', expand=True, fill='x', padx=5)
tk.Button(btn_frame, text="Ejecutar Rutina", bg="gold", command=play_routine).pack(side='left', expand=True, fill='x', padx=5)
tk.Button(btn_frame, text="Limpiar", command=clear_routine).pack(side='left', expand=True, fill='x', padx=5)

routine_listbox = tk.Listbox(tab_routine, height=12)
routine_listbox.pack(fill='both', expand=True, padx=20, pady=10)

# Botón de Emergencia Global
tk.Button(root, text="Liberar Motores (Relajado)", bg="red", fg="white", font=("Arial", 12, "bold"), command=release_motors).pack(pady=15)

# Iniciar bucle de monitoreo en tiempo real
update_realtime_display()

root.mainloop()
