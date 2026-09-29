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

# --- Funciones de Validación (Límites Libres para pruebas empíricas) ---
def is_safe_cartesian(x, y, z):
    return True, "Posición permitida."

def is_safe_angles(j2, j3):
    return True, "Ángulos permitidos."

# --- Funciones de Control ---
def update_angles(val=None):
    if mc:
        angles = [s.get() for s in angle_sliders]
        mc.send_angles(angles, speed_var.get())

def send_current_coords():
    if mc:
        coords = [s.get() for s in coord_sliders]
        mc.send_coords(coords, speed_var.get(), 0) # Modo 0: Articular (MoveJ)

def send_text_coords():
    if mc:
        try:
            coords = [float(var.get()) for var in text_coord_vars]
            mc.send_coords(coords, speed_var.get(), 0) # Modo 0: Articular (MoveJ)
        except ValueError:
            messagebox.showerror("Error", "Ingresa únicamente valores numéricos.")

def go_home():
    if mc:
        mc.send_angles([0, 0, 0, 0, 0, 0], speed_var.get())
        for slider in angle_sliders:
            slider.set(0)

def release_motors():
    if mc:
        mc.release_all_servos()

# --- Funciones de Rutina y Puntos Predefinidos ---
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

def move_to_mapped_point(angles):
    """Envía los ángulos directamente para una fiabilidad absoluta del 100%"""
    if mc:
        mc.send_angles(angles, speed_var.get())
        # Actualizamos también los sliders de la pestaña 1 visualmente
        for slider, angle in zip(angle_sliders, angles):
            slider.set(angle)

# --- Función de Monitoreo en Tiempo Real ---
def update_realtime_display():
    if mc:
        try:
            coords = mc.get_coords()
            if coords and len(coords) == 6:
                for i, val in enumerate(coords):
                    realtime_labels[i].config(text=f"{coord_labels[i]}: {val:.2f}")
        except Exception:
            pass
    root.after(500, update_realtime_display)

# --- Interfaz Gráfica ---
root = tk.Tk()
root.title("MyCobot 320 Pi - Control Optimizado")
root.geometry("600x750")

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
notebook.add(tab_coords, text="Coord (Sliders)")
coord_limits = [(-350, 350), (-350, 350), (-41, 523.9), (-180, 180), (-180, 180), (-180, 180)]
coord_labels = ["X", "Y", "Z", "RX", "RY", "RZ"]
coord_sliders = []
tk.Label(tab_coords, text="Ajusta los valores y presiona 'Mover'", fg="gray").pack(pady=5)
for i, limits in enumerate(coord_limits):
    frame = tk.Frame(tab_coords)
    frame.pack(fill='x', padx=10, pady=5)
    tk.Label(frame, text=f"{coord_labels[i]}:", width=5).pack(side='left')
    slider = tk.Scale(frame, from_=limits[0], to=limits[1], orient='horizontal', length=400)
    slider.set(200 if coord_labels[i] == 'Z' else 0)
    slider.pack(side='right')
    coord_sliders.append(slider)
tk.Button(tab_coords, text="Mover a Coordenadas", bg="orange", command=send_current_coords).pack(pady=15)

# Pestaña 3: Coordenadas por Texto y Tiempo Real
tab_text_coords = ttk.Frame(notebook)
notebook.add(tab_text_coords, text="Coord (Texto)")
columns_frame = tk.Frame(tab_text_coords)
columns_frame.pack(fill='both', expand=True, padx=10, pady=10)

left_col = tk.Frame(columns_frame)
left_col.pack(side='left', fill='both', expand=True)
tk.Label(left_col, text="Posición Actual", font=("Arial", 10, "bold")).pack(pady=5)
realtime_labels = []
for label_text in coord_labels:
    lbl = tk.Label(left_col, text=f"{label_text}: 0.00", font=("Arial", 11))
    lbl.pack(anchor='w', pady=8, padx=20)
    realtime_labels.append(lbl)

right_col = tk.Frame(columns_frame)
right_col.pack(side='right', fill='both', expand=True)
tk.Label(right_col, text="Ingresar Coordenadas", font=("Arial", 10, "bold")).pack(pady=5)
text_coord_vars = []
for i, label_text in enumerate(coord_labels):
    f = tk.Frame(right_col)
    f.pack(fill='x', pady=5, padx=10)
    tk.Label(f, text=f"{label_text}:", width=4).pack(side='left')
    var = tk.StringVar(value="200" if label_text == 'Z' else "0")
    entry = tk.Entry(f, textvariable=var, width=12, justify='center')
    entry.pack(side='right')
    text_coord_vars.append(var)
tk.Button(right_col, text="Aplicar Movimiento", bg="orange", command=send_text_coords).pack(pady=20)

# --- NUEVA PESTAÑA: PUNTOS MAPEADOS ---
tab_mapped = ttk.Frame(notebook)
notebook.add(tab_mapped, text="Puntos Mapeados")

tk.Label(tab_mapped, text="Movimiento basado en Ángulos Articulares (Alta Fiabilidad)", font=("Arial", 10, "bold")).pack(pady=10)
tk.Label(tab_mapped, text="Al ejecutar, el robot irá a los ángulos registrados,\nresultando en las coordenadas X,Y,Z esperadas.", justify="center", fg="gray").pack(pady=5)

# Diccionario con los datos extraídos de la terminal
mapped_points = {
    "P1": [9.66, -49.48, -63.89, 5.53, 99.22, 0.26],
    "P2": [12.48, -52.82, -91.4, 69.87, 95.8, -0.08],
    "P3": [5.27, -65.47, -81.91, 81.82, 109.51, 1.14],
    "P4": [13.18, -59.58, -93.77, 61.96, 80.77, 1.14]
}

# Crear botones para cada punto mapeado
for point_name, angles in mapped_points.items():
    frame_btn = tk.Frame(tab_mapped)
    frame_btn.pack(fill='x', padx=40, pady=8)
    btn = tk.Button(frame_btn, text=f"Ir a {point_name}", bg="lightgreen", font=("Arial", 11, "bold"), 
                    command=lambda a=angles: move_to_mapped_point(a))
    btn.pack(side='left', fill='x', expand=True)
    tk.Label(frame_btn, text=f"Ángulos: {angles}", font=("Arial", 8)).pack(side='right', padx=10)

# Pestaña 5: Rutinas Libres
tab_routine = ttk.Frame(notebook)
notebook.add(tab_routine, text="Rutinas")
tk.Label(tab_routine, text="1. Libera los motores.\n2. Mueve el brazo manualmente.\n3. Guarda los puntos.", justify="left").pack(pady=10)
btn_frame = tk.Frame(tab_routine)
btn_frame.pack(fill='x', padx=20)
tk.Button(btn_frame, text="Guardar Punto", bg="lightgreen", command=save_point).pack(side='left', expand=True, fill='x', padx=5)
tk.Button(btn_frame, text="Ejecutar", bg="gold", command=play_routine).pack(side='left', expand=True, fill='x', padx=5)
tk.Button(btn_frame, text="Limpiar", command=clear_routine).pack(side='left', expand=True, fill='x', padx=5)
routine_listbox = tk.Listbox(tab_routine, height=10)
routine_listbox.pack(fill='both', expand=True, padx=20, pady=10)

# Botón de Emergencia Global
tk.Button(root, text="Liberar Motores (Relajado)", bg="red", fg="white", font=("Arial", 12, "bold"), command=release_motors).pack(pady=10)

update_realtime_display()
root.mainloop()
