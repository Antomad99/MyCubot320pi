import tkinter as tk
from tkinter import ttk
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

# --- Funciones de Control ---
def update_angles(val=None):
    if mc:
        angles = [s.get() for s in angle_sliders]
        mc.send_angles(angles, speed_var.get())

def update_coords(val=None):
    if mc:
        coords = [s.get() for s in coord_sliders]
        mc.send_coords(coords, speed_var.get(), 1)

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
        # Lee la posición real del brazo (útil si lo moviste con la mano tras liberarlo)
        current_angles = mc.get_angles()
        if current_angles:
            routine_points.append(current_angles)
            routine_listbox.insert(tk.END, f"P {len(routine_points)}: {current_angles}")

def play_routine(index=0):
    if mc and index < len(routine_points):
        # Envía el brazo al punto guardado
        mc.send_angles(routine_points[index], speed_var.get())
        
        # Estima un tiempo de espera seguro (2500 ms) antes de enviar el siguiente punto
        # En un entorno avanzado, usarías sync_send_angles en un hilo separado
        root.after(2500, play_routine, index + 1)

def clear_routine():
    routine_points.clear()
    routine_listbox.delete(0, tk.END)


# --- Interfaz Gráfica ---
root = tk.Tk()
root.title("MyCobot 320 Pi - Control Avanzado")
root.geometry("500x650")

# Panel Global (Velocidad y Origen)
global_frame = tk.Frame(root)
global_frame.pack(fill='x', pady=10, padx=20)

tk.Label(global_frame, text="Velocidad Global:", font=("Arial", 10, "bold")).pack(side='left')
speed_var = tk.IntVar(value=40)
speed_slider = tk.Scale(global_frame, from_=1, to=100, orient='horizontal', variable=speed_var, length=150)
speed_slider.pack(side='left', padx=10)

tk.Button(global_frame, text="Ir al Origen (Home)", bg="lightblue", command=go_home).pack(side='right')

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
    slider = tk.Scale(frame, from_=limits[0], to=limits[1], orient='horizontal', length=350, command=update_angles)
    slider.set(0)
    slider.pack(side='right')
    angle_sliders.append(slider)

# Pestaña 2: Coordenadas
tab_coords = ttk.Frame(notebook)
notebook.add(tab_coords, text="Coordenadas")
coord_limits = [(-350, 350), (-350, 350), (-41, 523.9), (-180, 180), (-180, 180), (-180, 180)]
coord_labels = ["X", "Y", "Z", "RX", "RY", "RZ"]
coord_sliders = []
for i, limits in enumerate(coord_limits):
    frame = tk.Frame(tab_coords)
    frame.pack(fill='x', padx=10, pady=5)
    tk.Label(frame, text=f"{coord_labels[i]}:", width=5).pack(side='left')
    slider = tk.Scale(frame, from_=limits[0], to=limits[1], orient='horizontal', length=350, command=update_coords)
    slider.set(200 if coord_labels[i] == 'Z' else 0)
    slider.pack(side='right')
    coord_sliders.append(slider)

# Pestaña 3: Rutinas
tab_routine = ttk.Frame(notebook)
notebook.add(tab_routine, text="Rutinas")
tk.Label(tab_routine, text="1. Libera los motores.\n2. Mueve el brazo manualmente.\n3. Guarda los puntos para crear una secuencia.", justify="left").pack(pady=10)

btn_frame = tk.Frame(tab_routine)
btn_frame.pack(fill='x', padx=20)
tk.Button(btn_frame, text="Guardar Punto", bg="lightgreen", command=save_point).pack(side='left', expand=True, fill='x', padx=5)
tk.Button(btn_frame, text="Ejecutar Rutina", bg="gold", command=play_routine).pack(side='left', expand=True, fill='x', padx=5)
tk.Button(btn_frame, text="Limpiar", command=clear_routine).pack(side='left', expand=True, fill='x', padx=5)

routine_listbox = tk.Listbox(tab_routine, height=15)
routine_listbox.pack(fill='both', expand=True, padx=20, pady=10)

# Botón de Emergencia
tk.Button(root, text="Liberar Motores (Relajado)", bg="red", fg="white", font=("Arial", 12, "bold"), command=release_motors).pack(pady=15)

root.mainloop()