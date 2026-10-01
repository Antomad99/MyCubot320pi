import tkinter as tk
from tkinter import ttk, messagebox
import math
import time
from pymycobot import MyCobot320

# Integración de Matplotlib para la interfaz
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

PORT = '/dev/ttyAMA0'
BAUD = 115200

try:
    mc = MyCobot320(PORT, BAUD)
    mc.power_on()
except Exception as e:
    print(f"Error de conexión: {e}")
    mc = None

routine_points = []
mapping_waypoints = []
saved_map_points = []
mapping_index = 0
is_mapping = False
is_updating_sliders = False

# --- Funciones de Control Básicas ---
def update_angles(val=None):
    if is_updating_sliders: return
    if mc:
        angles = [s.get() for s in angle_sliders]
        mc.send_angles(angles, speed_var.get())

def sync_interface():
    global is_updating_sliders
    if not mc: return
    try:
        angles = mc.get_angles()
        coords = mc.get_coords()
        is_updating_sliders = True
        if angles and len(angles) == 6:
            for slider, val in zip(angle_sliders, angles): slider.set(val)
        if coords and len(coords) == 6:
            for slider, val in zip(coord_sliders, coords): slider.set(val)
        is_updating_sliders = False
        messagebox.showinfo("Sincronización Exitosa", "Postura actualizada.")
    except Exception as e:
        is_updating_sliders = False
        messagebox.showerror("Error", f"Fallo al sincronizar: {e}")

def send_current_coords():
    if mc:
        coords = [s.get() for s in coord_sliders]
        mc.send_coords(coords, speed_var.get(), 0) 

def send_text_coords():
    if mc:
        try:
            coords = [float(var.get()) for var in text_coord_vars]
            mc.send_coords(coords, speed_var.get(), 0) 
        except ValueError:
            messagebox.showerror("Error", "Valores numéricos requeridos.")

def go_home():
    if mc:
        mc.send_angles([0, 0, 0, 0, 0, 0], speed_var.get())
        global is_updating_sliders
        is_updating_sliders = True
        for slider in angle_sliders: slider.set(0)
        is_updating_sliders = False

def release_motors():
    if mc: mc.release_all_servos()

def energize_motors():
    if mc:
        mc.power_on()
        current_angles = mc.get_angles()
        if current_angles and len(current_angles) == 6:
            mc.send_angles(current_angles, speed_var.get())

# --- Funciones de Mapeo y Ploteo ---
def start_mapping():
    global mapping_waypoints, mapping_index, is_mapping
    if not mc: return
    
    current_coords = mc.get_coords()
    if not current_coords or len(current_coords) != 6:
        messagebox.showerror("Error", "No se pudo leer la posición inicial.")
        return

    try:
        puntos_x = int(grid_x_var.get())
        puntos_y = int(grid_y_var.get())
        paso_x = float(step_x_var.get())
        paso_y = float(step_y_var.get())
        delay_ms = int(delay_var.get())
    except ValueError:
        messagebox.showerror("Error", "Formatos de malla inválidos.")
        return

    x0, y0, z0, rx, ry, rz = current_coords
    mapping_waypoints.clear()
    
    # Generar trayectoria de malla
    for j in range(puntos_y):
        rango_x = range(puntos_x) if j % 2 == 0 else reversed(range(puntos_x))
        for i in rango_x:
            mapping_waypoints.append([x0 + (i * paso_x), y0 + (j * paso_y), z0, rx, ry, rz])

    mapping_index = 0
    is_mapping = True
    btn_start_map.config(state=tk.DISABLED)
    btn_stop_map.config(state=tk.NORMAL)
    
    # Configurar Gráfico
    ax.clear()
    xs = [p[0] for p in mapping_waypoints]
    ys = [p[1] for p in mapping_waypoints]
    ax.plot(xs, ys, 'k--', alpha=0.5, label="Ruta Planeada")
    ax.set_title("Plano de Mapeo XY")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    canvas.draw()

    execute_mapping_step(delay_ms)

def execute_mapping_step(delay_ms):
    global mapping_index, is_mapping
    if not is_mapping: return

    if mapping_index < len(mapping_waypoints):
        punto = mapping_waypoints[mapping_index]
        if mc: mc.send_coords(punto, speed_var.get(), 1) 
        
        # Actualizar Gráfico
        ax.scatter(punto[0], punto[1], c='blue')
        canvas.draw()
        
        lbl_mapping_status.config(text=f"Mapeando: {mapping_index + 1}/{len(mapping_waypoints)}\nX: {punto[0]:.2f} | Y: {punto[1]:.2f}")
        mapping_index += 1
        root.after(delay_ms, lambda: execute_mapping_step(delay_ms))
    else:
        is_mapping = False
        lbl_mapping_status.config(text="Mapeo completado.")
        btn_start_map.config(state=tk.NORMAL)
        btn_stop_map.config(state=tk.DISABLED)

def stop_mapping():
    global is_mapping
    is_mapping = False
    lbl_mapping_status.config(text="Detenido.")
    btn_start_map.config(state=tk.NORMAL)
    btn_stop_map.config(state=tk.DISABLED)

def save_current_map_point():
    if not mc: return
    coords = mc.get_coords()
    if coords:
        saved_map_points.append(coords)
        map_listbox.insert(tk.END, f"P{len(saved_map_points)}: X:{coords[0]:.1f}, Y:{coords[1]:.1f}")
        # Marcar en el gráfico
        ax.scatter(coords[0], coords[1], c='red', marker='x', s=100)
        canvas.draw()

def go_to_saved_map_point():
    selection = map_listbox.curselection()
    if selection and mc:
        idx = selection[0]
        coords = saved_map_points[idx]
        mc.send_coords(coords, speed_var.get(), 1)

# --- Función de Laparoscopía (Movimiento Cónico RCM) ---
def execute_laparoscopy_cone():
    if not mc: return
    try:
        xp = float(pivot_x_var.get())
        yp = float(pivot_y_var.get())
        zp = float(pivot_z_var.get())
        R = float(cone_radius_var.get())
        h = float(cone_height_var.get())
        steps = int(cone_steps_var.get())
    except ValueError:
        messagebox.showerror("Error", "Valores numéricos inválidos para el cono.")
        return

    # Generar trayectoria
    cone_trajectory = []
    for i in range(steps):
        theta = (2 * math.pi / steps) * i
        
        # Posición del TCP
        x_tcp = xp + R * math.cos(theta)
        y_tcp = yp + R * math.sin(theta)
        z_tcp = zp + h
        
        # Cálculo de Orientación Euler para apuntar al pivote
        # Asumiendo convención XYZ básica, ajustamos Pitch(RY) y Roll(RX)
        ry = math.degrees(math.atan2(x_tcp - xp, z_tcp - zp))
        rx = math.degrees(math.atan2(y_tcp - yp, z_tcp - zp))
        rz = 0.0 # Mantenemos la rotación Z constante
        
        cone_trajectory.append([x_tcp, y_tcp, z_tcp, rx, ry, rz])
    
    # Ejecutar trayectoria
    for point in cone_trajectory:
        mc.send_coords(point, speed_var.get(), 1)
        time.sleep(0.5) # Pausa entre interpolaciones
    messagebox.showinfo("Laparoscopía", "Movimiento cónico RCM completado.")

# --- Interfaz Gráfica ---
root = tk.Tk()
root.title("MyCobot 320 Pi - Control Académico Avanzado")
root.geometry("900x900")

global_frame = tk.Frame(root)
global_frame.pack(fill='x', pady=10, padx=20)
tk.Label(global_frame, text="Velocidad:", font=("Arial", 10, "bold")).pack(side='left')
speed_var = tk.IntVar(value=40)
tk.Scale(global_frame, from_=1, to=100, orient='horizontal', variable=speed_var, length=120).pack(side='left', padx=5)
tk.Button(global_frame, text="Sincronizar Interfaz", command=sync_interface).pack(side='left', padx=15)
tk.Button(global_frame, text="Ir al Origen", command=go_home).pack(side='right')

notebook = ttk.Notebook(root)
notebook.pack(pady=10, expand=True, fill='both')

# Pestañas Articulaciones y Coord (Simplificadas en esta vista para brevedad de estructura)
tab_angles = ttk.Frame(notebook)
notebook.add(tab_angles, text="Articulaciones")
angle_limits = [(-168, 168), (-135, 135), (-145, 145), (-148, 148), (-168, 168), (-180, 180)]
angle_sliders = []
for i, limits in enumerate(angle_limits):
    frame = tk.Frame(tab_angles)
    frame.pack(fill='x', padx=10, pady=5)
    tk.Label(frame, text=f"J{i+1}:", width=5).pack(side='left')
    slider = tk.Scale(frame, from_=limits[0], to=limits[1], orient='horizontal', length=400, command=update_angles)
    slider.pack(side='right')
    angle_sliders.append(slider)

tab_coords = ttk.Frame(notebook)
notebook.add(tab_coords, text="Coord (Sliders)")
coord_limits = [(-350, 350), (-350, 350), (-41, 523.9), (-180, 180), (-180, 180), (-180, 180)]
coord_labels = ["X", "Y", "Z", "RX", "RY", "RZ"]
coord_sliders = []
for i, limits in enumerate(coord_limits):
    frame = tk.Frame(tab_coords)
    frame.pack(fill='x', padx=10, pady=5)
    tk.Label(frame, text=f"{coord_labels[i]}:", width=5).pack(side='left')
    slider = tk.Scale(frame, from_=limits[0], to=limits[1], orient='horizontal', length=400)
    slider.pack(side='right')
    coord_sliders.append(slider)
tk.Button(tab_coords, text="Mover a Coordenadas", command=send_current_coords).pack(pady=15)

# --- Pestaña: Mapeo 2D y Visualización ---
tab_mapping = ttk.Frame(notebook)
notebook.add(tab_mapping, text="Mapeo & Visualización")

# Panel Izquierdo: Controles
map_ctrl_frame = tk.Frame(tab_mapping)
map_ctrl_frame.pack(side='left', fill='y', padx=10, pady=10)

tk.Label(map_ctrl_frame, text="Config. Malla", font=("Arial", 12, "bold")).pack(pady=5)
grid_x_var, grid_y_var = tk.StringVar(value="3"), tk.StringVar(value="3")
step_x_var, step_y_var = tk.StringVar(value="20.0"), tk.StringVar(value="20.0")
delay_var = tk.StringVar(value="3000")

tk.Label(map_ctrl_frame, text="Puntos X / Y:").pack()
tk.Entry(map_ctrl_frame, textvariable=grid_x_var, width=5).pack()
tk.Entry(map_ctrl_frame, textvariable=grid_y_var, width=5).pack()
tk.Label(map_ctrl_frame, text="Pasos X / Y (mm):").pack()
tk.Entry(map_ctrl_frame, textvariable=step_x_var, width=5).pack()
tk.Entry(map_ctrl_frame, textvariable=step_y_var, width=5).pack()

btn_start_map = tk.Button(map_ctrl_frame, text="Iniciar Mapeo", bg="lightgreen", command=start_mapping)
btn_start_map.pack(pady=5)
btn_stop_map = tk.Button(map_ctrl_frame, text="Detener", bg="salmon", command=stop_mapping, state=tk.DISABLED)
btn_stop_map.pack()
lbl_mapping_status = tk.Label(map_ctrl_frame, text="Listo.")
lbl_mapping_status.pack(pady=10)

tk.Button(map_ctrl_frame, text="Guardar Punto Actual", bg="lightblue", command=save_current_map_point).pack(pady=10)
map_listbox = tk.Listbox(map_ctrl_frame, height=8)
map_listbox.pack(fill='x')
tk.Button(map_ctrl_frame, text="Ir al Punto Seleccionado", command=go_to_saved_map_point).pack(pady=5)

# Panel Derecho: Gráfico Matplotlib
fig = Figure(figsize=(5, 5), dpi=100)
ax = fig.add_subplot(111)
ax.set_title("Plano de Mapeo XY")
canvas = FigureCanvasTkAgg(fig, master=tab_mapping)
canvas.get_tk_widget().pack(side='right', fill='both', expand=True)

# --- Pestaña: Laparoscopía (Cono RCM) ---
tab_laparo = ttk.Frame(notebook)
notebook.add(tab_laparo, text="Laparoscopía (RCM)")

tk.Label(tab_laparo, text="Simulación de Centro de Movimiento Remoto", font=("Arial", 12, "bold")).pack(pady=15)

laparo_frame = tk.Frame(tab_laparo)
laparo_frame.pack(pady=10)

pivot_x_var, pivot_y_var, pivot_z_var = tk.StringVar(value="200"), tk.StringVar(value="0"), tk.StringVar(value="100")
cone_radius_var, cone_height_var = tk.StringVar(value="50"), tk.StringVar(value="150")
cone_steps_var = tk.StringVar(value="12")

tk.Label(laparo_frame, text="Punto de Incisión (Pivote RCM) X, Y, Z:").grid(row=0, column=0, pady=5)
tk.Entry(laparo_frame, textvariable=pivot_x_var, width=6).grid(row=0, column=1)
tk.Entry(laparo_frame, textvariable=pivot_y_var, width=6).grid(row=0, column=2)
tk.Entry(laparo_frame, textvariable=pivot_z_var, width=6).grid(row=0, column=3)

tk.Label(laparo_frame, text="Radio del Cono en TCP (mm):").grid(row=1, column=0, pady=5)
tk.Entry(laparo_frame, textvariable=cone_radius_var, width=6).grid(row=1, column=1)

tk.Label(laparo_frame, text="Altura TCP sobre Pivote (mm):").grid(row=2, column=0, pady=5)
tk.Entry(laparo_frame, textvariable=cone_height_var, width=6).grid(row=2, column=1)

tk.Label(laparo_frame, text="Resolución (pasos por vuelta):").grid(row=3, column=0, pady=5)
tk.Entry(laparo_frame, textvariable=cone_steps_var, width=6).grid(row=3, column=1)

tk.Button(tab_laparo, text="Ejecutar Cono Laparoscópico", bg="orange", font=("Arial", 11, "bold"), command=execute_laparoscopy_cone).pack(pady=20)

# Botones de Energía
power_frame = tk.Frame(root)
power_frame.pack(fill='x', pady=10, padx=20)
tk.Button(power_frame, text="Liberar Motores", bg="red", fg="white", command=release_motors).pack(side='left', expand=True, fill='x', padx=5)
tk.Button(power_frame, text="Energizar Motores", bg="green", fg="white", command=energize_motors).pack(side='right', expand=True, fill='x', padx=5)

root.mainloop()
