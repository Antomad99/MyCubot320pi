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
    # 1. Piso Virtual (Eje Z)
    Z_MIN = 40  # mm - Altura mínima de seguridad sobre la base
    if z < Z_MIN:
        return False, f"Violación de límite: Z ({z} mm) es menor al mínimo de seguridad ({Z_MIN} mm)."

    # 2. Extensión Máxima (Radio en el plano XY)
    RADIO_MAX = 315  # mm - Alcance físico máximo del myCobot 320
    radio_actual = math.sqrt(x**2 + y**2)
    if radio_actual > RADIO_MAX:
        return False, f"Fuera de rango: El radio ({radio_actual:.1f} mm) excede la extensión máxima permitida."

    # 3. Cilindro de Exclusión Central (Evita golpear la propia base J1)
    RADIO_MIN_EXCLUSION = 95  # mm - Grosor del robot + margen
    ALTURA_CRITICA = 160      # mm - Altura del pilar central
    if radio_actual < RADIO_MIN_EXCLUSION and z < ALTURA_CRITICA:
         return False, "Riesgo de colisión: El efector está invadiendo el cilindro central de la base."

    return True, "Posición segura."

def is_safe_angles(j2, j3):
    """
    Reglas heurísticas para cinemática directa.
    Previene pliegues peligrosos entre el hombro (J2) y el codo (J3).
    """
    # Si el hombro apunta hacia abajo y el codo se cierra hacia adentro, choca con la base
    if j2 < -45 and j3 > 90:
        return False, "Geometría riesgosa: Inclinación del hombro y pliegue del codo provocan auto-colisión."
    return True, "Ángulos seguros."


# --- Funciones de Control ---

def update_angles(val=None):
    if mc:
        angles = [s.get() for s in angle_sliders]
        
        # Validar la geometría articular antes de enviar
        seguro, mensaje = is_safe_angles(angles[1], angles[2])
        if seguro:
            mc.send_angles(angles, speed_var.get())
        else:
            # En cinemática directa por slider, imprimimos en consola para no saturar con pop-ups
            print(f"BLOQUEO PREVENTIVO: {mensaje}")

def send_current_coords():
    """Se ejecuta al presionar el botón de Mover a Coordenadas"""
    if mc:
        coords = [s.get() for s in coord_sliders]
        x, y, z = coords[0], coords[1], coords[2]
        
        # Validar matemáticamente la coordenada solicitada
        seguro, mensaje = is_safe_cartesian(x, y, z)
        
        if seguro:
            # Enviar coordenada de forma lineal (modo 1)
            mc.send_coords(coords, speed_var.get(), 1)
        else:
            # Emitir alerta en pantalla y no mover el robot
            messagebox.showwarning("Límite de Seguridad Activado", mensaje)

def go_home():
    if mc:
        # Enviar al cero seguro
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


# --- Interfaz Gráfica ---

root = tk.Tk()
root.title("MyCobot 320 Pi - Control Seguro")
root.geometry("550x700")

# Panel Global (Velocidad y Origen)
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

# Pestaña 2: Coordenadas (Con botón de ejecución seguro)
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
    # Sin el 'command', no envía datos al arrastrar
    slider = tk.Scale(frame, from_=limits[0], to=limits[1], orient='horizontal', length=400)
    slider.set(200 if coord_labels[i] == 'Z' else 0)
    slider.pack(side='right')
    coord_sliders.append(slider)

tk.Button(tab_coords, text="Mover a Coordenadas", bg="orange", font=("Arial", 11, "bold"), command=send_current_coords).pack(pady=15)

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

root.mainloop()