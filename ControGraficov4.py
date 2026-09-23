import tkinter as tk
from tkinter import ttk, messagebox
import math
from pymycobot import MyCobot320

PORT = '/dev/ttyAMA0'
BAUD = 115200

# === NUEVO: configuración de la herramienta (extensión / "pinza laparoscópica") ===
# TOOL_LENGTH_MM: distancia en mm desde la brida (flange) del robot hasta la punta
# real de tu extensión, medida a lo largo del eje de la herramienta.
# AJUSTA ESTE VALOR midiendo tu extensión físicamente antes de usar el mapeo.
TOOL_LENGTH_MM = 150

# Límites de inserción (distancia entre el punto de trócar y la punta objetivo).
MIN_INSERTION_MM = 20   # evita que la punta quede pegada al trócar
MAX_INSERTION_MM = 220  # evita pedir una inserción más larga que tu extensión física

# Ángulo máximo de inclinación del eje de inserción respecto al eje "nominal"
# (calculado del trócar al centro del plano), para evitar posturas extremas.
MAX_TILT_DEG = 55

routine_points = []

# === NUEVO: estado del mapeo de plano ===
pivot_point = None       # [x, y, z] del punto de trócar, capturado con get_coords()
plane_corners = []       # lista de hasta 4 [x, y, z] capturados en orden: 0=origen,1=+u,2=opuesto,3=+v
plane_grid = []          # lista de filas; cada fila es lista de dicts {x,y,z}
nominal_axis = None      # eje de referencia (trócar -> centro del plano) para chequear inclinación

try:
    mc = MyCobot320(PORT, BAUD)
    mc.power_on()
    if TOOL_LENGTH_MM > 0:
        # Redefine el TCP en la punta de la extensión y hace que send_coords/get_coords
        # trabajen en el sistema de coordenadas de la herramienta, no de la brida.
        mc.set_tool_reference([0, 0, TOOL_LENGTH_MM, 0, 0, 0])
        mc.set_end_type(1)  # 1 = coordenadas referidas a la herramienta (TCP)
except Exception as e:
    print(f"Error de conexión: {e}")
    mc = None

# --- Funciones de Validación Matemática (originales) ---

def is_safe_cartesian(x, y, z):
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
    if j2 < -45 and j3 > 90:
        return False, "Geometría riesgosa: Inclinación del hombro y pliegue del codo provocan auto-colisión."
    return True, "Ángulos seguros."

# === NUEVO: álgebra vectorial mínima (sin numpy, para no depender de más librerías en el Pi) ===

def v_sub(a, b):
    return [a[i] - b[i] for i in range(3)]

def v_add(a, b):
    return [a[i] + b[i] for i in range(3)]

def v_scale(a, s):
    return [a[i] * s for i in range(3)]

def v_dot(a, b):
    return sum(a[i] * b[i] for i in range(3))

def v_cross(a, b):
    return [
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0],
    ]

def v_norm(a):
    return math.sqrt(v_dot(a, a))

def v_normalize(a):
    n = v_norm(a)
    if n < 1e-9:
        raise ValueError("Vector de longitud casi cero, no se puede normalizar.")
    return [a[i] / n for i in range(3)]

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

# === NUEVO: validación de inserción respecto al trócar ===

def is_safe_insertion(pivot, target):
    if pivot is None:
        return False, "No se ha capturado el punto de trócar (pivote)."
    d = v_norm(v_sub(target, pivot))
    if d < MIN_INSERTION_MM:
        return False, f"Inserción ({d:.1f} mm) menor al mínimo seguro ({MIN_INSERTION_MM} mm)."
    if d > MAX_INSERTION_MM:
        return False, f"Inserción ({d:.1f} mm) excede el máximo permitido ({MAX_INSERTION_MM} mm)."

    if nominal_axis is not None:
        dir_vec = v_normalize(v_sub(target, pivot))
        cos_ang = clamp(v_dot(dir_vec, nominal_axis), -1.0, 1.0)
        ang_deg = math.degrees(math.acos(cos_ang))
        if ang_deg > MAX_TILT_DEG:
            return False, f"Inclinación del instrumento ({ang_deg:.1f}°) excede el máximo permitido ({MAX_TILT_DEG}°)."

    return True, "Inserción segura."

# === NUEVO: orientación por pivote fijo (Remote Center of Motion) ===
#
# ADVERTENCIA IMPORTANTE:
# La conversión de matriz de rotación a (RX, RY, RZ) asume la convención de ángulos
# fijos Rz * Ry * Rx, que es la más común en estos brazos, pero pymycobot / MyCobot320
# no documentan explícitamente el orden y signo exactos. ANTES de usar esto en modo
# automático, corre verify_orientation_convention() (ver abajo) y confirma visualmente
# que la punta apunta hacia el trócar como se espera. Si está invertido o rotado,
# ajusta la función euler_from_matrix() en consecuencia.

def euler_from_matrix(R):
    """R es una lista 3x3 (filas). Devuelve (rx, ry, rz) en grados."""
    sy = -R[2][0]
    sy = clamp(sy, -1.0, 1.0)
    ry = math.asin(sy)
    cy = math.cos(ry)
    if abs(cy) > 1e-6:
        rx = math.atan2(R[2][1], R[2][2])
        rz = math.atan2(R[1][0], R[0][0])
    else:
        # Gimbal lock
        rx = math.atan2(-R[1][2], R[1][1])
        rz = 0.0
    return math.degrees(rx), math.degrees(ry), math.degrees(rz)

def compute_rcm_orientation(pivot, target):
    """
    Calcula RX,RY,RZ tales que el eje Z de la herramienta (dirección de inserción)
    apunte desde el trócar hacia el punto objetivo.
    """
    z_axis = v_normalize(v_sub(target, pivot))

    up_ref = [0, 0, 1]
    if abs(v_dot(up_ref, z_axis)) > 0.95:
        up_ref = [1, 0, 0]

    x_axis = v_normalize(v_cross(up_ref, z_axis))
    y_axis = v_cross(z_axis, x_axis)

    # Matriz de rotación con columnas [x_axis, y_axis, z_axis]
    R = [
        [x_axis[0], y_axis[0], z_axis[0]],
        [x_axis[1], y_axis[1], z_axis[1]],
        [x_axis[2], y_axis[2], z_axis[2]],
    ]
    return euler_from_matrix(R)

def verify_orientation_convention():
    """
    Ayuda de calibración: manda la punta a un punto de prueba directamente
    'hacia abajo' desde un trócar hipotético y reporta los ángulos calculados,
    para que compares contra lo que observas físicamente en el robot.
    """
    test_pivot = [0, 0, 300]
    test_target = [0, 0, 100]  # target directamente debajo del pivote
    rx, ry, rz = compute_rcm_orientation(test_pivot, test_target)
    print(f"[Calibración] Para inserción vertical hacia abajo -> RX={rx:.1f}, RY={ry:.1f}, RZ={rz:.1f}")
    print("Compara esto enviando manualmente esos ángulos con los sliders de la pestaña Coordenadas")
    print("y verifica que la herramienta quede orientada verticalmente. Ajusta euler_from_matrix() si no coincide.")

# --- Funciones de Control (originales) ---

def update_angles(val=None):
    if mc:
        angles = [s.get() for s in angle_sliders]
        seguro, mensaje = is_safe_angles(angles[1], angles[2])
        if seguro:
            mc.send_angles(angles, speed_var.get())
        else:
            print(f"BLOQUEO PREVENTIVO: {mensaje}")

def send_current_coords():
    if mc:
        coords = [s.get() for s in coord_sliders]
        x, y, z = coords[0], coords[1], coords[2]
        seguro, mensaje = is_safe_cartesian(x, y, z)
        if seguro:
            mc.send_coords(coords, speed_var.get(), 1)
        else:
            messagebox.showwarning("Límite de Seguridad Activado", mensaje)

def go_home():
    if mc:
        mc.send_angles([0, 0, 0, 0, 0, 0], speed_var.get())
        for slider in angle_sliders:
            slider.set(0)

def release_motors():
    if mc:
        mc.release_all_servos()

# --- Funciones de Rutina (originales) ---

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

# === NUEVO: funciones de mapeo de plano por trócar ===

def capture_pivot():
    global pivot_point
    if not mc:
        messagebox.showerror("Sin conexión", "El robot no está conectado.")
        return
    coords = mc.get_coords()
    if not coords:
        messagebox.showerror("Error", "No se pudo leer la posición actual del robot.")
        return
    pivot_point = coords[:3]
    pivot_label_var.set(f"Trócar: [{pivot_point[0]:.1f}, {pivot_point[1]:.1f}, {pivot_point[2]:.1f}]")

def capture_corner():
    if not mc:
        messagebox.showerror("Sin conexión", "El robot no está conectado.")
        return
    if len(plane_corners) >= 4:
        messagebox.showinfo("Límite alcanzado", "Ya capturaste 4 esquinas. Usa 'Reiniciar Esquinas' para empezar de nuevo.")
        return
    coords = mc.get_coords()
    if not coords:
        messagebox.showerror("Error", "No se pudo leer la posición actual del robot.")
        return
    plane_corners.append(coords[:3])
    idx = len(plane_corners)
    corners_listbox.insert(tk.END, f"Esquina {idx}: [{coords[0]:.1f}, {coords[1]:.1f}, {coords[2]:.1f}]")

def reset_corners():
    plane_corners.clear()
    corners_listbox.delete(0, tk.END)
    plane_grid.clear()
    draw_grid_canvas()

def bilinear(u, v, c0, c1, c2, c3):
    """c0=origen, c1=+u, c2=esquina opuesta, c3=+v (orden cíclico alrededor del plano)."""
    p = [0.0, 0.0, 0.0]
    for i in range(3):
        p[i] = ((1-u)*(1-v)*c0[i] + u*(1-v)*c1[i] + u*v*c2[i] + (1-u)*v*c3[i])
    return p

def compute_grid():
    global plane_grid, nominal_axis

    if pivot_point is None:
        messagebox.showwarning("Falta el trócar", "Primero captura el punto de trócar (pivote).")
        return
    if len(plane_corners) < 3:
        messagebox.showwarning("Faltan esquinas", "Captura al menos 3 esquinas del plano (idealmente 4).")
        return

    corners = list(plane_corners)
    if len(corners) == 3:
        c0, c1, c3 = corners
        c2 = [c1[i] + c3[i] - c0[i] for i in range(3)]  # asume rectángulo/paralelogramo
    else:
        c0, c1, c2, c3 = corners[:4]

    try:
        rows = int(rows_var.get())
        cols = int(cols_var.get())
    except ValueError:
        messagebox.showerror("Valor inválido", "Filas y columnas deben ser enteros.")
        return
    if rows < 1 or cols < 1:
        messagebox.showerror("Valor inválido", "Filas y columnas deben ser mayores a 0.")
        return

    center = bilinear(0.5, 0.5, c0, c1, c2, c3)
    try:
        nominal_axis = v_normalize(v_sub(center, pivot_point))
    except ValueError:
        messagebox.showerror("Error geométrico", "El trócar coincide con el centro del plano; revisa las capturas.")
        return

    grid = []
    for i in range(rows):
        v = i / (rows - 1) if rows > 1 else 0.0
        row = []
        for j in range(cols):
            u = j / (cols - 1) if cols > 1 else 0.0
            p = bilinear(u, v, c0, c1, c2, c3)
            row.append({"x": p[0], "y": p[1], "z": p[2]})
        grid.append(row)

    plane_grid = grid
    draw_grid_canvas()
    status_var.set(f"Malla generada: {rows}x{cols} puntos.")

def move_to_point(p):
    if not mc:
        messagebox.showerror("Sin conexión", "El robot no está conectado.")
        return
    target = [p["x"], p["y"], p["z"]]

    seguro, mensaje = is_safe_insertion(pivot_point, target)
    if not seguro:
        messagebox.showwarning("Límite de Trócar Activado", mensaje)
        return

    seguro, mensaje = is_safe_cartesian(*target)
    if not seguro:
        messagebox.showwarning("Límite de Espacio de Trabajo Activado", mensaje)
        return

    rx, ry, rz = compute_rcm_orientation(pivot_point, target)
    coords6 = target + [rx, ry, rz]
    mc.send_coords(coords6, speed_var.get(), 1)
    status_var.set(f"Punta -> [{target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f}]  RX={rx:.1f} RY={ry:.1f} RZ={rz:.1f}")

def sweep_grid(flat_index=0):
    flat_points = [p for row in plane_grid for p in row]
    if not flat_points:
        messagebox.showwarning("Sin malla", "Primero calcula la malla del plano.")
        return
    if flat_index < len(flat_points):
        move_to_point(flat_points[flat_index])
        root.after(2000, sweep_grid, flat_index + 1)
    else:
        status_var.set("Barrido completo.")

CANVAS_SIZE = 320

def draw_grid_canvas():
    grid_canvas.delete("all")
    if not plane_grid:
        return
    rows = len(plane_grid)
    cols = len(plane_grid[0])
    cell_w = CANVAS_SIZE / cols
    cell_h = CANVAS_SIZE / rows
    for i in range(rows):
        for j in range(cols):
            x0, y0 = j * cell_w, i * cell_h
            x1, y1 = x0 + cell_w, y0 + cell_h
            grid_canvas.create_rectangle(x0, y0, x1, y1, fill="#d9ecff", outline="white", tags=f"cell_{i}_{j}")
    grid_canvas.create_text(CANVAS_SIZE/2, 10, text="Click en una celda para mover la punta ahí", fill="gray")

def on_canvas_click(event):
    if not plane_grid:
        return
    rows = len(plane_grid)
    cols = len(plane_grid[0])
    cell_w = CANVAS_SIZE / cols
    cell_h = CANVAS_SIZE / rows
    j = int(event.x // cell_w)
    i = int(event.y // cell_h)
    if 0 <= i < rows and 0 <= j < cols:
        move_to_point(plane_grid[i][j])

# --- Interfaz Gráfica ---

root = tk.Tk()
root.title("MyCobot 320 Pi - Control Seguro")
root.geometry("600x800")

global_frame = tk.Frame(root)
global_frame.pack(fill='x', pady=10, padx=20)

tk.Label(global_frame, text="Velocidad:", font=("Arial", 10, "bold")).pack(side='left')
speed_var = tk.IntVar(value=40)
speed_slider = tk.Scale(global_frame, from_=1, to=100, orient='horizontal', variable=speed_var, length=150)
speed_slider.pack(side='left', padx=10)

tk.Button(global_frame, text="Ir al Origen", bg="lightblue", command=go_home).pack(side='right')

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

# Pestaña 2: Coordenadas (IK)
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

# === NUEVO: Pestaña 3: Mapeo de Plano (Trócar / RCM) ===
tab_mapeo = ttk.Frame(notebook)
notebook.add(tab_mapeo, text="Mapeo (Trócar)")

tk.Label(
    tab_mapeo,
    text=("1. Libera motores y coloca la punta EN el orificio de entrada (trócar) -> Capturar Trócar.\n"
          "2. Vuelve a bloquear/mover con cuidado y toca 3-4 esquinas del plano dentro de la caja,\n"
          "   en orden: origen, +u, opuesta, +v -> Capturar Esquina (una vez por esquina).\n"
          "3. Define filas/columnas y Calcular Malla. 4. Click en la rejilla o usa Barrer Plano."),
    justify="left", fg="gray"
).pack(pady=8, padx=10, anchor='w')

pivot_frame = tk.Frame(tab_mapeo)
pivot_frame.pack(fill='x', padx=10, pady=5)
tk.Button(pivot_frame, text="Capturar Trócar (Pivote)", bg="#ffcc80", command=capture_pivot).pack(side='left')
pivot_label_var = tk.StringVar(value="Trócar: no capturado")
tk.Label(pivot_frame, textvariable=pivot_label_var).pack(side='left', padx=10)

corners_btn_frame = tk.Frame(tab_mapeo)
corners_btn_frame.pack(fill='x', padx=10, pady=5)
tk.Button(corners_btn_frame, text="Capturar Esquina", bg="lightgreen", command=capture_corner).pack(side='left')
tk.Button(corners_btn_frame, text="Reiniciar Esquinas", command=reset_corners).pack(side='left', padx=10)

corners_listbox = tk.Listbox(tab_mapeo, height=4)
corners_listbox.pack(fill='x', padx=10, pady=5)

grid_cfg_frame = tk.Frame(tab_mapeo)
grid_cfg_frame.pack(fill='x', padx=10, pady=5)
tk.Label(grid_cfg_frame, text="Filas:").pack(side='left')
rows_var = tk.StringVar(value="5")
tk.Entry(grid_cfg_frame, textvariable=rows_var, width=4).pack(side='left', padx=5)
tk.Label(grid_cfg_frame, text="Columnas:").pack(side='left')
cols_var = tk.StringVar(value="5")
tk.Entry(grid_cfg_frame, textvariable=cols_var, width=4).pack(side='left', padx=5)
tk.Button(grid_cfg_frame, text="Calcular Malla", bg="#90caf9", command=compute_grid).pack(side='left', padx=10)
tk.Button(grid_cfg_frame, text="Barrer Plano", bg="gold", command=lambda: sweep_grid(0)).pack(side='left')

grid_canvas = tk.Canvas(tab_mapeo, width=CANVAS_SIZE, height=CANVAS_SIZE, bg="white")
grid_canvas.pack(pady=10)
grid_canvas.bind("<Button-1>", on_canvas_click)

status_var = tk.StringVar(value="Sin mapear.")
tk.Label(tab_mapeo, textvariable=status_var, fg="blue").pack(pady=5)

# Pestaña 4: Rutinas
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

tk.Button(root, text="Liberar Motores (Relajado)", bg="red", fg="white", font=("Arial", 12, "bold"), command=release_motors).pack(pady=15)

if __name__ == "__main__":
    # Descomenta la siguiente línea la primera vez para calibrar la convención de ángulos:
    # verify_orientation_convention()
    root.mainloop()
