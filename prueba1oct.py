import tkinter as tk
from tkinter import ttk, messagebox
import math
import time
import json
import os
from pymycobot import MyCobot320

# Integración de Matplotlib para la visualización espacial
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

# Archivo de persistencia de puntos
POINTS_FILE = "puntos_guardados.json"

# Puntos predefinidos por defecto
default_angle_points = [
    [9.66, -49.48, -63.89, 5.53, 99.22, 0.26],
    [12.48, -52.82, -91.4, 69.87, 95.8, -0.08],
    [5.27, -65.47, -81.91, 81.82, 109.51, 1.14],
    [13.18, -59.58, -93.77, 61.96, 80.77, 1.14],
]
saved_angle_points = []

# Carga y guardado de puntos fijos
def load_points():
    global saved_angle_points
    if os.path.exists(POINTS_FILE):
        try:
            with open(POINTS_FILE, 'r') as f:
                saved_angle_points = json.load(f)
        except:
            saved_angle_points = default_angle_points.copy()
    else:
        saved_angle_points = default_angle_points.copy()
        save_points_to_file()

def save_points_to_file():
    with open(POINTS_FILE, 'w') as f:
        json.dump(saved_angle_points, f)

load_points()

# Variables globales para control de estado
routine_points = []
mapping_waypoints = []
mapping_index = 0
is_mapping = False
is_updating_sliders = False
user_is_interacting = False # Seguro para manipular sliders sin interrupción

def on_slider_press(event):
    global user_is_interacting
    user_is_interacting = True

def on_slider_release(event):
    global user_is_interacting
    user_is_interacting = False

# --- Funciones de Control y Sincronización ---
def update_angles(val=None):
    if is_updating_sliders:
        return
    if mc and user_is_interacting:
        angles = [s.get() for s in angle_sliders]
        mc.send_angles(angles, speed_var.get())

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
            messagebox.showerror("Error", "Ingresa únicamente valores numéricos.")

def go_home():
    if mc:
        mc.send_angles([0, 0, 0, 0, 0, 0], speed_var.get())

def release_motors():
    if mc:
        mc.release_all_servos()

def energize_motors():
    if mc:
        mc.power_on()
        current_angles = mc.get_angles()
        if current_angles and len(current_angles) == 6:
            mc.send_angles(current_angles, speed_var.get())

def update_realtime_display():
    """Monitoreo continuo del robot y actualización automática de los sliders"""
    global is_updating_sliders
    if mc:
        try:
            coords = mc.get_coords()
            angles = mc.get_angles()
            
            # Actualización del panel superior de lectura
            if coords and len(coords) == 6:
                for i, val in enumerate(coords):
                    realtime_labels[i].config(text=f"{coord_labels[i]}: {val:.2f}")
            
            # Actualización en vivo de los sliders (sólo si no los estamos tocando)
            if not user_is_interacting:
                is_updating_sliders = True
                if angles and len(angles) == 6:
                    for i, val in enumerate(angles):
                        angle_sliders[i].set(val)
                if coords and len(coords) == 6:
                    for i, val in enumerate(coords):
                        coord_sliders[i].set(val)
                is_updating_sliders = False
        except Exception:
            is_updating_sliders = False
    root.after(500, update_realtime_display)

# --- Funciones de Rutinas y Puntos ---
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
    if mc:
        mc.send_angles(angles, speed_var.get())

# Funciones específicas para Pestaña 3 (Puntos Fijos y Persistentes)
def refresh_fixed_points_listbox():
    fixed_points_listbox.delete(0, tk.END)
    for i, angles in enumerate(saved_angle_points):
        fixed_points_listbox.insert(tk.END, f"P{i+1}: {angles}")

def add_new_angle_point():
    if mc:
        current_angles = mc.get_angles()
        if current_angles and len(current_angles) == 6:
            saved_angle_points.append(current_angles)
            save_points_to_file()
            refresh_fixed_points_listbox()

def delete_selected_angle_point():
    selection = fixed_points_listbox.curselection()
    if selection:
        idx = selection[0]
        del saved_angle_points[idx]
        save_points_to_file()
        refresh_fixed_points_listbox()

def go_to_selected_angle_point():
    selection = fixed_points_listbox.curselection()
    if selection and mc:
        idx = selection[0]
        angles = saved_angle_points[idx]
        move_to_mapped_point(angles)

# --- Funciones de Barrido Rectangular 2D (Sólo Extremos) ---
def start_sweep():
    global mapping_waypoints, mapping_index, is_mapping
    if not mc: return
    
    current_coords = mc.get_coords()
    if not current_coords or len(current_coords) != 6:
        messagebox.showerror("Error", "No se pudo leer la posición inicial.")
        return

    try:
        width_x = float(rect_width_var.get())
        length_y = float(rect_length_var.get())
        delay_ms = int(delay_var.get())
    except ValueError:
        messagebox.showerror("Error", "Asegúrate de ingresar números válidos para el rectángulo.")
        return

    x0, y0, z0, rx, ry, rz = current_coords
    mapping_waypoints.clear()
    
    # Generar trayectoria únicamente a los 4 extremos (esquinas) y regresar al inicio
    mapping_waypoints.append([x0, y0, z0, rx, ry, rz])                      # 1: Inicio
    mapping_waypoints.append([x0 + width_x, y0, z0, rx, ry, rz])            # 2: Extremo X
    mapping_waypoints.append([x0 + width_x, y0 + length_y, z0, rx, ry, rz]) # 3: Extremo XY
    mapping_waypoints.append([x0, y0 + length_y, z0, rx, ry, rz])           # 4: Extremo Y
    mapping_waypoints.append([x0, y0, z0, rx, ry, rz])                      # 5: Regreso al origen para cerrar cuadro

    mapping_index = 0
    is_mapping = True
    btn_start_map.config(state=tk.DISABLED)
    btn_stop_map.config(state=tk.NORMAL)
    
    # Preparar el gráfico de Matplotlib
    ax.clear()
    xs = [p[0] for p in mapping_waypoints]
    ys = [p[1] for p in mapping_waypoints]
    ax.plot(xs, ys, 'k-', alpha=0.5, label="Contorno Rectangular")
    ax.scatter([x0], [y0], c='green', marker='o', s=100, label="Inicio")
    ax.set_title("Recorrido por Extremos XY")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.legend()
    ax.grid(True)
    canvas.draw()

    execute_sweep_step(delay_ms)

def execute_sweep_step(delay_ms):
    global mapping_index, is_mapping
    if not is_mapping: return

    if mapping_index < len(mapping_waypoints):
        punto = mapping_waypoints[mapping_index]
        if mc: mc.send_coords(punto, speed_var.get(), 1) 
        
        # Actualizar ploteo con la posición actual
        ax.scatter(punto[0], punto[1], c='blue')
        canvas.draw()
        
        lbl_mapping_status.config(text=f"Avanzando: Punto {mapping_index + 1}/{len(mapping_waypoints)}\nX: {punto[0]:.2f} | Y: {punto[1]:.2f}")
        mapping_index += 1
        root.after(delay_ms, lambda: execute_sweep_step(delay_ms))
    else:
        is_mapping = False
        lbl_mapping_status.config(text="Recorrido completado exitosamente.")
        btn_start_map.config(state=tk.NORMAL)
        btn_stop_map.config(state=tk.DISABLED)

def stop_sweep():
    global is_mapping
    is_mapping = False
    lbl_mapping_status.config(text="Recorrido detenido por el usuario.")
    btn_start_map.config(state=tk.NORMAL)
    btn_stop_map.config(state=tk.DISABLED)

# --- Algoritmo de Laparoscopía (RCM) ---
def execute_laparoscopy_cone():
    """Genera una trayectoria cónica manteniendo un punto de incisión fijo (RCM) sin bloquear la GUI"""
    if not mc: return
    try:
        xp = float(pivot_x_var.get())
        yp = float(pivot_y_var.get())
        zp = float(pivot_z_var.get())
        R = float(cone_radius_var.get())
        h = float(cone_height_var.get())
        steps = int(cone_steps_var.get())
    except ValueError:
        messagebox.showerror("Error", "Valores numéricos inválidos para la cinemática del cono.")
        return

    cone_trajectory = []
    for i in range(steps):
        theta = (2 * math.pi / steps) * i
        x_tcp = xp + R * math.cos(theta)
        y_tcp = yp + R * math.sin(theta)
        z_tcp = zp + h
        
        ry = math.degrees(math.atan2(x_tcp - xp, z_tcp - zp))
        rx = math.degrees(math.atan2(y_tcp - yp, z_tcp - zp))
        rz = 0.0 
        
        cone_trajectory.append([x_tcp, y_tcp, z_tcp, rx, ry, rz])
    
    def step_cone(idx):
        if idx < len(cone_trajectory):
            if mc: mc.send_coords(cone_trajectory[idx], speed_var.get(), 1)
            root.after(500, lambda: step_cone(idx + 1))
        else:
            messagebox.showinfo("Simulación Completada", "Trayectoria cónica RCM finalizada.")

    step_cone(0)


# ==========================================
# --- CONSTRUCCIÓN DE LA INTERFAZ GRÁFICA ---
# ==========================================
root = tk.Tk()
root.title("MyCobot 320 Pi - Control Académico Avanzado")
root.geometry("850x950")

# Panel Global (Velocidad, Origen)
global_frame = tk.Frame(root)
global_frame.pack(fill='x', pady=5, padx=20)

tk.Label(global_frame, text="Velocidad:", font=("Arial", 10, "bold")).pack(side='left')
speed_var = tk.IntVar(value=40)
speed_slider = tk.Scale(global_frame, from_=1, to=100, orient='horizontal', variable=speed_var, length=120)
speed_slider.pack(side='left', padx=5)

tk.Button(global_frame, text="Ir al Origen", bg="lightgray", command=go_home).pack(side='right')

# --- Panel Global de Posición en Tiempo Real ---
coord_labels = ["X", "Y", "Z", "RX", "RY", "RZ"]
realtime_frame = tk.Frame(root, bd=2, relief="groove")
realtime_frame.pack(fill='x', pady=5, padx=20)

tk.Label(realtime_frame, text="Posición Actual:", font=("Arial", 10, "bold")).pack(side='left', padx=10)
realtime_labels = []
for label_text in coord_labels:
    lbl = tk.Label(realtime_frame, text=f"{label_text}: 0.00", font=("Arial", 10, "bold"), fg="blue")
    lbl.pack(side='left', padx=10)
    realtime_labels.append(lbl)

# Contenedor de Pestañas
notebook = ttk.Notebook(root)
notebook.pack(pady=10, expand=True, fill='both')

# --- Pestaña 1: Articulaciones ---
tab_angles = ttk.Frame(notebook)
notebook.add(tab_angles, text="Articulaciones")
angle_limits = [(-168, 168), (-135, 135), (-145, 145), (-148, 148), (-168, 168), (-180, 180)]
angle_sliders = []
for i, limits in enumerate(angle_limits):
    frame = tk.Frame(tab_angles)
    frame.pack(fill='x', padx=10, pady=5)
    tk.Label(frame, text=f"J{i+1}:", width=5).pack(side='left')
    slider = tk.Scale(frame, from_=limits[0], to=limits[1], orient='horizontal', length=400, command=update_angles, resolution=0.1)
    slider.bind("<" + "ButtonPress-1" + ">", on_slider_press)
    slider.bind("<" + "ButtonRelease-1" + ">", on_slider_release)
    slider.set(0)
    slider.pack(side='right')
    angle_sliders.append(slider)

# --- Pestaña 2: Coordenadas Unificadas (Sliders + Texto) ---
tab_coords = ttk.Frame(notebook)
notebook.add(tab_coords, text="Coordenadas")

# Mitad superior: Sliders
coords_slider_frame = tk.Frame(tab_coords)
coords_slider_frame.pack(fill='x', pady=5)
tk.Label(coords_slider_frame, text="Control por Deslizador", font=("Arial", 11, "bold")).pack(pady=5)
coord_limits = [(-350, 350), (-350, 350), (-41, 523.9), (-180, 180), (-180, 180), (-180, 180)]
coord_sliders = []
for i, limits in enumerate(coord_limits):
    frame = tk.Frame(coords_slider_frame)
    frame.pack(fill='x', padx=50, pady=2)
    tk.Label(frame, text=f"{coord_labels[i]}:", width=5).pack(side='left')
    slider = tk.Scale(frame, from_=limits[0], to=limits[1], orient='horizontal', length=400, resolution=0.1)
    slider.bind("<" + "ButtonPress-1" + ">", on_slider_press)
    slider.bind("<" + "ButtonRelease-1" + ">", on_slider_release)
    slider.set(200 if coord_labels[i] == 'Z' else 0)
    slider.pack(side='right')
    coord_sliders.append(slider)
tk.Button(coords_slider_frame, text="Mover a Sliders", bg="orange", command=send_current_coords).pack(pady=10)

ttk.Separator(tab_coords, orient='horizontal').pack(fill='x', pady=5)

# Mitad inferior: Cajas de Texto
coords_text_frame = tk.Frame(tab_coords)
coords_text_frame.pack(fill='x', pady=5)
tk.Label(coords_text_frame, text="Control por Texto Exacto", font=("Arial", 11, "bold")).pack(pady=5)
text_inputs_container = tk.Frame(coords_text_frame)
text_inputs_container.pack()
text_coord_vars = []
for i, label_text in enumerate(coord_labels):
    f = tk.Frame(text_inputs_container)
    f.grid(row=i//3, column=i%3, padx=15, pady=5)
    tk.Label(f, text=f"{label_text}:", width=4).pack(side='left')
    var = tk.StringVar(value="200.0" if label_text == 'Z' else "0.0")
    entry = tk.Entry(f, textvariable=var, width=10, justify='center')
    entry.pack(side='right')
    text_coord_vars.append(var)
tk.Button(coords_text_frame, text="Aplicar Coordenadas", bg="orange", command=send_text_coords).pack(pady=10)


# --- Pestaña 3: Puntos Fijos (Persistentes) ---
tab_mapped = ttk.Frame(notebook)
notebook.add(tab_mapped, text="Puntos Fijos")

tk.Label(tab_mapped, text="Posiciones Guardadas (Persistentes)", font=("Arial", 12, "bold")).pack(pady=10)

btn_frame_fijos = tk.Frame(tab_mapped)
btn_frame_fijos.pack(fill='x', padx=20, pady=5)
tk.Button(btn_frame_fijos, text="Guardar Postura Actual", bg="lightblue", command=add_new_angle_point).pack(side='left', expand=True, fill='x', padx=5)
tk.Button(btn_frame_fijos, text="Eliminar Seleccionado", bg="salmon", command=delete_selected_angle_point).pack(side='left', expand=True, fill='x', padx=5)
tk.Button(btn_frame_fijos, text="Ir a Posición", bg="lightgreen", font=("Arial", 10, "bold"), command=go_to_selected_angle_point).pack(side='left', expand=True, fill='x', padx=5)

fixed_points_listbox = tk.Listbox(tab_mapped, height=15)
fixed_points_listbox.pack(fill='both', expand=True, padx=20, pady=10)
refresh_fixed_points_listbox() # Llenar la lista inicial

# --- Pestaña 4: Barrido Rectangular y Visualización Matplotlib ---
tab_mapping = ttk.Frame(notebook)
notebook.add(tab_mapping, text="Barrido Rectangular")

map_ctrl_frame = tk.Frame(tab_mapping)
map_ctrl_frame.pack(side='left', fill='y', padx=10, pady=10)

tk.Label(map_ctrl_frame, text="Config. de Extremos (XY)", font=("Arial", 10, "bold")).pack(pady=5)
rect_width_var = tk.StringVar(value="60.0")
rect_length_var = tk.StringVar(value="60.0")
delay_var = tk.StringVar(value="2000")

tk.Label(map_ctrl_frame, text="Ancho X (mm):").pack()
tk.Entry(map_ctrl_frame, textvariable=rect_width_var, width=8).pack()
tk.Label(map_ctrl_frame, text="Largo Y (mm):").pack()
tk.Entry(map_ctrl_frame, textvariable=rect_length_var, width=8).pack()
tk.Label(map_ctrl_frame, text="Pausa en Esquinas (ms):").pack()
tk.Entry(map_ctrl_frame, textvariable=delay_var, width=8).pack()

btn_start_map = tk.Button(map_ctrl_frame, text="Iniciar Recorrido", bg="lightgreen", command=start_sweep)
btn_start_map.pack(pady=10)
btn_stop_map = tk.Button(map_ctrl_frame, text="Detener", bg="salmon", command=stop_sweep, state=tk.DISABLED)
btn_stop_map.pack()
lbl_mapping_status = tk.Label(map_ctrl_frame, text="Esperando instrucciones...", fg="gray")
lbl_mapping_status.pack(pady=10)

# Gráfico de Matplotlib
fig = Figure(figsize=(5, 5), dpi=100)
ax = fig.add_subplot(111)
ax.set_title("Plano de Barrido XY")
ax.grid(True)
canvas = FigureCanvasTkAgg(fig, master=tab_mapping)
canvas.get_tk_widget().pack(side='right', fill='both', expand=True, padx=10, pady=10)

# --- Pestaña 5: Rutinas (Enseñanza Manual) ---
tab_routine = ttk.Frame(notebook)
notebook.add(tab_routine, text="Rutinas")
tk.Label(tab_routine, text="Protocolo de Enseñanza:\n1. Libera motores.\n2. Mueve el robot manualmente a la pose deseada.\n3. Guarda el punto.\n4. Energiza y ejecuta.", justify="left").pack(pady=10)
btn_frame = tk.Frame(tab_routine)
btn_frame.pack(fill='x', padx=20)
tk.Button(btn_frame, text="Guardar Pose Actual", bg="lightgreen", command=save_point).pack(side='left', expand=True, fill='x', padx=5)
tk.Button(btn_frame, text="Ejecutar Rutina", bg="gold", command=play_routine).pack(side='left', expand=True, fill='x', padx=5)
tk.Button(btn_frame, text="Limpiar Lista", command=clear_routine).pack(side='left', expand=True, fill='x', padx=5)
routine_listbox = tk.Listbox(tab_routine, height=12)
routine_listbox.pack(fill='both', expand=True, padx=20, pady=10)

# --- Pestaña 6: Laparoscopía (RCM) ---
tab_laparo = ttk.Frame(notebook)
notebook.add(tab_laparo, text="Laparoscopía (RCM)")

tk.Label(tab_laparo, text="Simulación de Centro de Movimiento Remoto (RCM)", font=("Arial", 12, "bold")).pack(pady=15)
laparo_frame = tk.Frame(tab_laparo)
laparo_frame.pack(pady=10)

pivot_x_var, pivot_y_var, pivot_z_var = tk.StringVar(value="200.0"), tk.StringVar(value="0.0"), tk.StringVar(value="100.0")
cone_radius_var, cone_height_var = tk.StringVar(value="50.0"), tk.StringVar(value="150.0")
cone_steps_var = tk.StringVar(value="12")

tk.Label(laparo_frame, text="Punto de Incisión (Pivote RCM) X, Y, Z:").grid(row=0, column=0, pady=5, sticky='e')
tk.Entry(laparo_frame, textvariable=pivot_x_var, width=6).grid(row=0, column=1, padx=2)
tk.Entry(laparo_frame, textvariable=pivot_y_var, width=6).grid(row=0, column=2, padx=2)
tk.Entry(laparo_frame, textvariable=pivot_z_var, width=6).grid(row=0, column=3, padx=2)

tk.Label(laparo_frame, text="Radio del Cono en TCP (mm):").grid(row=1, column=0, pady=5, sticky='e')
tk.Entry(laparo_frame, textvariable=cone_radius_var, width=6).grid(row=1, column=1)

tk.Label(laparo_frame, text="Altura TCP sobre Pivote (mm):").grid(row=2, column=0, pady=5, sticky='e')
tk.Entry(laparo_frame, textvariable=cone_height_var, width=6).grid(row=2, column=1)

tk.Label(laparo_frame, text="Resolución (pasos por vuelta):").grid(row=3, column=0, pady=5, sticky='e')
tk.Entry(laparo_frame, textvariable=cone_steps_var, width=6).grid(row=3, column=1)

tk.Button(tab_laparo, text="Ejecutar Cono Laparoscópico", bg="orange", font=("Arial", 11, "bold"), command=execute_laparoscopy_cone).pack(pady=20)

# --- Botones de Control de Energía del Hardware ---
power_frame = tk.Frame(root)
power_frame.pack(fill='x', pady=10, padx=20)
tk.Button(power_frame, text="Liberar Motores", bg="red", fg="white", font=("Arial", 12, "bold"), command=release_motors).pack(side='left', expand=True, fill='x', padx=5)
tk.Button(power_frame, text="Energizar Motores", bg="green", fg="white", font=("Arial", 12, "bold"), command=energize_motors).pack(side='right', expand=True, fill='x', padx=5)

# Iniciar el hilo de actualización de la UI en tiempo real
update_realtime_display()
root.mainloop()
