import tkinter as tk
from tkinter import ttk
from pymycobot import MyCobot320 #

PORT = '/dev/ttyAMA0'
BAUD = 115200
SPEED = 40

try:
    mc = MyCobot320(PORT, BAUD) #
    mc.power_on() #
except Exception as e:
    print(f"Error: {e}")
    mc = None

def update_angles(val):
    if mc:
        angles = [s.get() for s in angle_sliders]
        mc.send_angles(angles, SPEED) #

def update_coords(val):
    if mc:
        coords = [s.get() for s in coord_sliders]
        # mode 1 para movimiento lineal
        mc.send_coords(coords, SPEED, 1) #

def release_motors():
    if mc:
        mc.release_all_servos() #

# Interfaz Principal
root = tk.Tk()
root.title("MyCobot 320 Pi - Interfaz de Control")
root.geometry("450x550")

notebook = ttk.Notebook(root)
notebook.pack(pady=10, expand=True, fill='both')

# --- Pestaña 1: Control Articular ---
tab_angles = ttk.Frame(notebook)
notebook.add(tab_angles, text="Articulaciones (J1-J6)")

# Límites extraídos de la documentación
angle_limits = [
    (-168, 168), # J1
    (-135, 135), # J2
    (-145, 145), # J3
    (-148, 148), # J4
    (-168, 168), # J5
    (-180, 180)  # J6
]

angle_sliders = []
for i, limits in enumerate(angle_limits):
    frame = tk.Frame(tab_angles)
    frame.pack(fill='x', padx=10, pady=5)
    tk.Label(frame, text=f"J{i+1}:", width=5).pack(side='left')
    slider = tk.Scale(frame, from_=limits[0], to=limits[1], orient='horizontal', length=300, command=update_angles)
    slider.set(0)
    slider.pack(side='right')
    angle_sliders.append(slider)

# --- Pestaña 2: Control Cartesiano ---
tab_coords = ttk.Frame(notebook)
notebook.add(tab_coords, text="Coordenadas (XYZ)")

# Límites extraídos de la documentación
coord_limits = [
    (-350, 350),   # X
    (-350, 350),   # Y
    (-41, 523.9),  # Z
    (-180, 180),   # RX
    (-180, 180),   # RY
    (-180, 180)    # RZ
]
coord_labels = ["X", "Y", "Z", "RX", "RY", "RZ"]

coord_sliders = []
for i, limits in enumerate(coord_limits):
    frame = tk.Frame(tab_coords)
    frame.pack(fill='x', padx=10, pady=5)
    tk.Label(frame, text=f"{coord_labels[i]}:", width=5).pack(side='left')
    slider = tk.Scale(frame, from_=limits[0], to=limits[1], orient='horizontal', length=300, command=update_coords)
    # Inicializar en una posición segura (Z = 200) para evitar que intente ir a (0,0,0) dentro de la base
    slider.set(200 if coord_labels[i] == 'Z' else 0) 
    slider.pack(side='right')
    coord_sliders.append(slider)

# --- Botón Global ---
tk.Button(root, text="Liberar Motores (Relajado)", bg="red", fg="white", font=("Arial", 12), command=release_motors).pack(pady=15)

root.mainloop()