"""
MyCobot 320 Pi - Control + base para simulador laparoscópico
------------------------------------------------------------
Requisitos: pip install numpy pymycobot --break-system-packages

Contenido:
  1. Cinemática directa (FK) con tabla DH  -> DEBES VALIDARLA contra get_coords()
  2. Cinemática inversa (IK) numérica (Levenberg-Marquardt / DLS)
  3. RCM virtual (punto de trocar fijo): yaw / tilt / roll / profundidad
  4. Capa de seguridad (límites articulares, z mínima, saltos grandes)
  5. GUI con tkinter (debounce en sliders, rutinas en hilo aparte, guardar/cargar JSON)
"""
import csv
import json
import math
import os
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

import numpy as np
from pymycobot import MyCobot320

# ----------------------------------------------------------------------------
# CONFIGURACIÓN
# ----------------------------------------------------------------------------
PORT, BAUD = '/dev/ttyAMA0', 115200

JOINT_LIMITS = [(-168, 168), (-135, 135), (-145, 145),
                (-148, 148), (-168, 168), (-180, 180)]
# --- Límites de SOFTWARE (solo se aplican si LIMITS["on"] es True) ---
# Por defecto están APAGADOS (modo exploración). El firmware del robot conserva
# sus propios límites articulares: si pides algo fuera de ellos, lo ignora o da error.
LIMITS = {"on": False}
Z_MIN = 20.0          # mm, plano de la mesa (ajústalo a tu montaje)
MAX_SPEED = 60        # tope de velocidad cuando los límites están activos
MAX_JOINT_JUMP = 40   # grados, máximo salto articular por comando IK (límites activos)
LOG_PATH = "limites_empiricos.csv"   # registro de cada intento RCM
TOOL_LEN = 150.0      # mm, de la brida del robot a la punta del instrumento
VALID_TOL_MM = 3.0    # error FK máximo aceptado para habilitar la IK
ORI_WEIGHT = 50.0     # mm/rad, pondera orientación vs posición en la IK

# Tabla DH (DH estándar): (offset_theta[deg], d[mm], a[mm], alpha[deg])
# !!! VALORES DE PARTIDA (por analogía con la familia myCobot). !!!
# Verifícalos con el manual oficial de Elephant Robotics del 320 y con el botón
# "Validar FK" en 3-4 posturas distintas antes de confiar en la IK.
DH = [
    (0,    173.9,   0,    90),
    (-90,  0,      -135,  0),
    (0,    0,      -120,  0),
    (-90,  88.78,   0,    90),
    (90,   95.0,    0,   -90),
    (0,    65.5,    0,    0),
]

# ----------------------------------------------------------------------------
# CINEMÁTICA
# ----------------------------------------------------------------------------
def _dh(theta, d, a, alpha):
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([[ct, -st * ca,  st * sa, a * ct],
                     [st,  ct * ca, -ct * sa, a * st],
                     [0,        sa,       ca,      d],
                     [0,         0,        0,      1]])


def fk(angles_deg):
    """Pose 4x4 de la brida en el marco base (mm)."""
    T = np.eye(4)
    for q, (off, d, a, al) in zip(angles_deg, DH):
        T = T @ _dh(math.radians(q + off), d, a, math.radians(al))
    return T


def rpy_to_R(rx, ry, rz):
    """Euler XYZ extrínseco (R = Rz·Ry·Rx), grados. Convención a CONFIRMAR con get_coords()."""
    rx, ry, rz = map(math.radians, (rx, ry, rz))
    Rx = np.array([[1, 0, 0], [0, math.cos(rx), -math.sin(rx)], [0, math.sin(rx), math.cos(rx)]])
    Ry = np.array([[math.cos(ry), 0, math.sin(ry)], [0, 1, 0], [-math.sin(ry), 0, math.cos(ry)]])
    Rz = np.array([[math.cos(rz), -math.sin(rz), 0], [math.sin(rz), math.cos(rz), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def R_to_rpy(R):
    ry = -math.asin(max(-1.0, min(1.0, R[2, 0])))
    if abs(math.cos(ry)) > 1e-6:
        rx = math.atan2(R[2, 1], R[2, 2])
        rz = math.atan2(R[1, 0], R[0, 0])
    else:  # bloqueo de cardán
        rx, rz = math.atan2(-R[1, 2], R[1, 1]), 0.0
    return [math.degrees(rx), math.degrees(ry), math.degrees(rz)]


def _rot_err(R, Rd):
    """Vector de rotación (eje·ángulo, rad) que lleva R a Rd. Válido para TODOS los ángulos,
    incluido 180° (la versión con productos cruz valía 0 a 180° y daba falsas convergencias)."""
    Re = Rd @ R.T
    th = math.acos(max(-1.0, min(1.0, (np.trace(Re) - 1.0) / 2.0)))
    v = np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
    if th < 1e-6:
        return 0.5 * v
    if th > math.pi - 1e-3:
        M = (Re + np.eye(3)) / 2.0
        i = int(np.argmax(np.diag(M)))
        n = M[:, i] / math.sqrt(max(M[i, i], 1e-12))
        return th * n
    return th / (2.0 * math.sin(th)) * v


def _pose_error(T, Td):
    ep = Td[:3, 3] - T[:3, 3]
    return np.hstack([ep, _rot_err(T[:3, :3], Td[:3, :3]) * ORI_WEIGHT])


def pivot_distance(T, trocar):
    """Distancia (mm) del trocar a la recta del instrumento (eje Z de la brida). Debe ser ~0."""
    return float(np.linalg.norm(np.cross(np.asarray(trocar, float) - T[:3, 3], T[:3, 2])))


def ik(Td, q0, iters=300, tol_mm=0.2, tol_rad=0.002, lam=0.05):
    """IK numérica (DLS). Devuelve ángulos en grados o None si no converge."""
    q = np.array(q0, dtype=float)
    lims = JOINT_LIMITS if LIMITS["on"] else [(-180, 180)] * 6
    lo = np.array([l[0] for l in lims], float)
    hi = np.array([l[1] for l in lims], float)
    h = 0.05  # grados, para el jacobiano numérico
    for _ in range(iters):
        e = _pose_error(fk(q), Td)
        if np.linalg.norm(e[:3]) < tol_mm and np.linalg.norm(e[3:]) / ORI_WEIGHT < tol_rad:
            return q
        J = np.zeros((6, 6))
        for i in range(6):
            dq = q.copy()
            dq[i] += h
            J[:, i] = (e - _pose_error(fk(dq), Td)) / math.radians(h)
        step = J.T @ np.linalg.solve(J @ J.T + (lam ** 2) * np.eye(6), e)
        step = np.degrees(step)
        step = np.clip(step, -5, 5)          # paso máximo por iteración
        q = np.clip(q + step, lo, hi)
    return None


def rcm_flange_pose(trocar, azimuth, tilt, roll, depth, tool_len=TOOL_LEN):
    """
    Pose de la brida para que el eje del instrumento pase SIEMPRE por 'trocar'.
      trocar  : (x,y,z) mm en marco base (el punto fijo en la pared abdominal)
      azimuth : giro del eje alrededor de la vertical (deg)
      tilt    : inclinación respecto a la vertical (deg). 0 = instrumento hacia abajo
      roll    : giro del instrumento sobre su propio eje (deg)
      depth   : mm que la punta ha entrado más allá del trocar (>= 0)
    Se asume que el eje del instrumento es el eje Z de la brida.
    """
    az, t = math.radians(azimuth), math.radians(tilt)
    u = np.array([math.sin(t) * math.cos(az), math.sin(t) * math.sin(az), -math.cos(t)])  # hacia adentro
    tip = np.asarray(trocar, float) + depth * u
    flange = tip - tool_len * u
    ref = np.array([1.0, 0, 0]) if abs(u[0]) < 0.9 else np.array([0, 1.0, 0])
    x = ref - ref.dot(u) * u
    x /= np.linalg.norm(x)
    y = np.cross(u, x)
    r = math.radians(roll)
    x, y = math.cos(r) * x + math.sin(r) * y, -math.sin(r) * x + math.cos(r) * y
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, y, u, flange
    return T, tip


def _rodrigues(w):
    th = float(np.linalg.norm(w))
    if th < 1e-12:
        return np.eye(3)
    k = w / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * (K @ K)


def interp_pose(Ta, Tb, s):
    """Posición lineal + orientación por rotación mínima (slerp)."""
    T = np.eye(4)
    T[:3, 3] = Ta[:3, 3] * (1 - s) + Tb[:3, 3] * s
    T[:3, :3] = _rodrigues(_rot_err(Ta[:3, :3], Tb[:3, :3]) * s) @ Ta[:3, :3]
    return T


def rcm_params_from_pose(T, trocar, tool_len=TOOL_LEN, fallback_az=0.0):
    """Inversa de rcm_flange_pose: (azimut, inclinación, roll, profundidad) de una pose y su
    distancia al pivote (mm). Si la distancia es grande, la pose NO está sobre el pivote."""
    u, f = T[:3, 2], T[:3, 3]
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, -u[2]))))
    az = math.degrees(math.atan2(u[1], u[0])) if tilt > 0.5 else fallback_az
    depth = float(np.dot(f + tool_len * u - np.asarray(trocar, float), u))
    ref = np.array([1.0, 0, 0]) if abs(u[0]) < 0.9 else np.array([0, 1.0, 0])
    xr = ref - ref.dot(u) * u
    xr /= np.linalg.norm(xr)
    yr = np.cross(u, xr)
    roll = math.degrees(math.atan2(T[:3, 0].dot(yr), T[:3, 0].dot(xr)))
    return (az, tilt, roll, depth), pivot_distance(T, trocar)


def _wrap(d):
    return (d + 180.0) % 360.0 - 180.0


def jacobian_condition(q):
    """Cond. del jacobiano (grande = cerca de una singularidad)."""
    h = 0.05
    T0 = fk(q)
    J = np.zeros((6, 6))
    for i in range(6):
        dq = np.array(q, float)
        dq[i] += h
        T1 = fk(dq)
        J[:3, i] = (T1[:3, 3] - T0[:3, 3]) / math.radians(h)
        J[3:, i] = _rot_err(T0[:3, :3], T1[:3, :3]) * ORI_WEIGHT / math.radians(h)
    sv = np.linalg.svd(J, compute_uv=False)
    return float(sv[0] / max(sv[-1], 1e-9))


def plan_to_rcm(q0, trocar, target, max_step=3.0, max_pivot_err=1.0, max_steps=640):
    """
    Planifica waypoints articulares desde q0 hasta la pose RCM 'target' = (az, tilt, roll, depth).
      - Si q0 ya está sobre el pivote: interpola los PARÁMETROS del RCM (el eje no abandona el pivote).
      - Si no (primer movimiento): interpola la pose de la brida (lineal + slerp).
    Cada IK parte de la solución anterior (continuidad de rama). Se subdivide hasta que ningún
    paso mueva una articulación más de 'max_step' grados. Devuelve (qs, info) o (None, info).
    """
    q0 = np.array(q0, float)
    Ta = fk(q0)
    Tb, _ = rcm_flange_pose(trocar, *target)
    pa, d0 = rcm_params_from_pose(Ta, trocar, fallback_az=target[0])
    on_pivot = d0 < 3.0
    n, reason = 8, ""
    while n <= max_steps:
        qs, worst_pivot, ok = [q0], 0.0, True
        for k in range(1, n + 1):
            sfrac = k / n
            if on_pivot:
                prm = (pa[0] + _wrap(target[0] - pa[0]) * sfrac,
                       pa[1] + (target[1] - pa[1]) * sfrac,
                       pa[2] + _wrap(target[2] - pa[2]) * sfrac,
                       pa[3] + (target[3] - pa[3]) * sfrac)
                T, _ = rcm_flange_pose(trocar, *prm)
            else:
                T = interp_pose(Ta, Tb, sfrac)
            q = ik(T, qs[-1], iters=120)
            if q is None:
                ok, reason = False, f"la IK no converge (tramo {sfrac:.0%}): pose inalcanzable o singular"
                break
            pv = pivot_distance(fk(q), trocar) if on_pivot else 0.0
            if pv > max_pivot_err:
                ok, reason = False, f"se pierde el pivote ({pv:.1f} mm) en el tramo {sfrac:.0%}"
                break
            jump = float(np.max(np.abs(q - qs[-1])))
            if jump > max_step:
                ok, reason = False, f"salto articular de {jump:.0f}° en el tramo {sfrac:.0%} (cambio de rama o singularidad)"
                break
            worst_pivot = max(worst_pivot, pv)
            qs.append(q)
        if ok:
            conds = [jacobian_condition(q) for q in qs[:: max(1, len(qs) // 10)]]
            return np.array(qs), {"steps": n, "on_pivot": on_pivot, "pivot_err": worst_pivot,
                                  "max_cond": max(conds), "reason": ""}
        if reason.startswith("la IK") and n >= 16:
            break   # no se arregla subdividiendo más
        n *= 2
    return None, {"reason": reason, "on_pivot": on_pivot}


# ----------------------------------------------------------------------------
# ROBOT (capa de seguridad + acceso serial con lock)
# ----------------------------------------------------------------------------
class Robot:
    def __init__(self):
        self.lock = threading.Lock()
        self.mc = None
        try:
            self.mc = MyCobot320(PORT, BAUD)
            self.mc.power_on()
        except Exception as e:
            print(f"Error de conexión: {e}")

    @property
    def ok(self):
        return self.mc is not None

    def angles(self):
        if not self.ok:
            return None
        with self.lock:
            a = self.mc.get_angles()
        return list(a) if isinstance(a, (list, tuple)) and len(a) == 6 else None

    def coords(self):
        if not self.ok:
            return None
        with self.lock:
            c = self.mc.get_coords()
        return list(c) if isinstance(c, (list, tuple)) and len(c) == 6 else None

    def send_angles(self, angles, speed):
        if not self.ok:
            return False
        if LIMITS["on"]:
            angles = [max(lo, min(hi, a)) for a, (lo, hi) in zip(angles, JOINT_LIMITS)]
            T = fk(angles)
            tip = T[:3, 3] + TOOL_LEN * T[:3, 2]
            if T[2, 3] < Z_MIN or tip[2] < Z_MIN:
                print("BLOQUEADO: la brida o la punta estarían por debajo de Z_MIN")
                return False
            speed = min(speed, MAX_SPEED)
        with self.lock:
            self.mc.send_angles([float(a) for a in angles], int(speed))
        return True

    def wait_arrival(self, timeout=15):
        time.sleep(0.3)
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self.lock:
                moving = self.mc.is_moving()
            if moving == 0:
                return True
            time.sleep(0.1)
        return False

    def release(self):
        if self.ok:
            with self.lock:
                self.mc.release_all_servos()

    def focus(self):
        if self.ok:
            with self.lock:
                self.mc.focus_all_servos()


robot = Robot()
ui_q = queue.Queue()          # los hilos de trabajo NO tocan Tk: dejan aquí sus actualizaciones
path_gen = {"n": 0}           # generación de trayectoria; una nueva orden cancela la anterior
routine_points = []
fk_validations = []          # errores (mm) de las validaciones FK hechas
routine_stop = threading.Event()

# ----------------------------------------------------------------------------
# GUI
# ----------------------------------------------------------------------------
root = tk.Tk()
root.title("MyCobot 320 Pi - Simulador Laparoscópico (base)")
root.geometry("640x820")

speed_var = tk.IntVar(value=30)
_debounce_ids = {}


def debounced(key, fn, delay=150):
    """Evita saturar el puerto serial mientras se arrastra un slider."""
    if key in _debounce_ids:
        root.after_cancel(_debounce_ids[key])
    _debounce_ids[key] = root.after(delay, fn)


# ---- Panel global
gf = tk.Frame(root)
gf.pack(fill='x', pady=8, padx=20)
tk.Label(gf, text="Velocidad:", font=("Arial", 10, "bold")).pack(side='left')
tk.Scale(gf, from_=1, to=100, orient='horizontal', variable=speed_var, length=150).pack(side='left', padx=10)
limits_var = tk.BooleanVar(value=False)
tk.Checkbutton(gf, text="Límites SW", variable=limits_var,
               command=lambda: LIMITS.update(on=limits_var.get())).pack(side='right')

status_var = tk.StringVar(value="Listo" if robot.ok else "SIN CONEXIÓN con el robot")
tk.Label(root, textvariable=status_var, fg="gray").pack()

nb = ttk.Notebook(root)
nb.pack(pady=6, expand=True, fill='both')

# ---- Pestaña 1: articulaciones
tab_j = ttk.Frame(nb)
nb.add(tab_j, text="Articulaciones")
angle_sliders = []
_updating_sliders = False  # evita que set() programático dispare comandos


def on_angle_slider(_=None):
    if _updating_sliders:
        return
    debounced("joints", lambda: robot.send_angles([s.get() for s in angle_sliders], speed_var.get()))


def set_sliders(angles):
    global _updating_sliders
    _updating_sliders = True
    for s, a in zip(angle_sliders, angles):
        s.set(a)
    _updating_sliders = False


for i, (lo, hi) in enumerate(JOINT_LIMITS):
    f = tk.Frame(tab_j)
    f.pack(fill='x', padx=10, pady=4)
    tk.Label(f, text=f"J{i+1}:", width=5).pack(side='left')
    s = tk.Scale(f, from_=lo, to=hi, orient='horizontal', length=400, command=on_angle_slider)
    s.set(0)
    s.pack(side='right')
    angle_sliders.append(s)


def go_home():
    set_sliders([0] * 6)
    robot.send_angles([0] * 6, speed_var.get())


tk.Button(tab_j, text="Ir al Origen", bg="lightblue", command=go_home).pack(pady=10)

# ---- Pestaña 2: coordenadas (firmware) + monitor
tab_c = ttk.Frame(nb)
nb.add(tab_c, text="Coordenadas")
coord_labels = ["X", "Y", "Z", "RX", "RY", "RZ"]
rt_labels, coord_vars = [], []
lc = tk.Frame(tab_c)
lc.pack(side='left', fill='both', expand=True, padx=10, pady=10)
tk.Label(lc, text="Posición actual", font=("Arial", 10, "bold")).pack()
for n in coord_labels:
    l = tk.Label(lc, text=f"{n}: 0.00", font=("Arial", 11))
    l.pack(anchor='w', pady=6, padx=15)
    rt_labels.append(l)
rc = tk.Frame(tab_c)
rc.pack(side='right', fill='both', expand=True, padx=10, pady=10)
tk.Label(rc, text="Ir a coordenadas (IK del firmware)", font=("Arial", 10, "bold")).pack()
for n in coord_labels:
    f = tk.Frame(rc)
    f.pack(fill='x', pady=4)
    tk.Label(f, text=f"{n}:", width=4).pack(side='left')
    v = tk.StringVar(value="200" if n == 'Z' else "0")
    tk.Entry(f, textvariable=v, width=10, justify='center').pack(side='right')
    coord_vars.append(v)


def send_text_coords():
    if not robot.ok:
        return
    try:
        c = [float(v.get()) for v in coord_vars]
    except ValueError:
        messagebox.showerror("Error", "Ingresa únicamente valores numéricos.")
        return
    if LIMITS["on"] and c[2] < Z_MIN:
        messagebox.showwarning("Bloqueado", f"Z debe ser >= {Z_MIN} mm")
        return
    spd = min(speed_var.get(), MAX_SPEED) if LIMITS["on"] else speed_var.get()
    with robot.lock:
        # mode 0 = angular (MoveJ), mode 1 = lineal (MoveL)
        robot.mc.send_coords(c, spd, 0)


tk.Button(rc, text="Aplicar", bg="orange", command=send_text_coords).pack(pady=15)


def update_realtime():
    c = robot.coords()
    if c:
        for i, val in enumerate(c):
            rt_labels[i].config(text=f"{coord_labels[i]}: {val:.2f}")
    root.after(500, update_realtime)


# ---- Pestaña 3: puntos mapeados
tab_m = ttk.Frame(nb)
nb.add(tab_m, text="Puntos")
mapped_points = {
    "P1": [9.66, -49.48, -63.89, 5.53, 99.22, 0.26],
    "P2": [12.48, -52.82, -91.4, 69.87, 95.8, -0.08],
    "P3": [5.27, -65.47, -81.91, 81.82, 109.51, 1.14],
    "P4": [13.18, -59.58, -93.77, 61.96, 80.77, 1.14],
}


def go_mapped(a):
    set_sliders(a)
    robot.send_angles(a, speed_var.get())


for name, a in mapped_points.items():
    f = tk.Frame(tab_m)
    f.pack(fill='x', padx=30, pady=6)
    tk.Button(f, text=f"Ir a {name}", bg="lightgreen", font=("Arial", 11, "bold"),
              command=lambda a=a: go_mapped(a)).pack(side='left', fill='x', expand=True)
    tk.Label(f, text=str(a), font=("Arial", 8)).pack(side='right', padx=8)

# ---- Pestaña 4: RCM laparoscópico
tab_r = ttk.Frame(nb)
nb.add(tab_r, text="RCM Laparoscopía")

tk.Label(tab_r, text="Trocar (punto fijo, mm en marco base)", font=("Arial", 10, "bold")).pack(pady=(8, 2))
tf = tk.Frame(tab_r)
tf.pack()
trocar_vars = []
for n, dv in zip("XYZ", ("250", "0", "150")):
    tk.Label(tf, text=n).pack(side='left')
    v = tk.StringVar(value=dv)
    tk.Entry(tf, textvariable=v, width=7, justify='center').pack(side='left', padx=4)
    trocar_vars.append(v)


def trocar_from_tip():
    """Pon la punta del instrumento (a mano, motores liberados) donde irá el trocar y pulsa."""
    a = robot.angles()
    if not a:
        return
    T = fk(a)
    tip = T[:3, 3] + TOOL_LEN * T[:3, 2]
    for v, val in zip(trocar_vars, tip):
        v.set(f"{val:.1f}")
    status_var.set("Trocar fijado en la punta actual (según FK)")


tk.Button(tab_r, text="Trocar = punta actual", command=trocar_from_tip).pack(pady=4)

rcm_sliders = {}
for key, lbl, lo, hi, init in [("azimuth", "Azimut", -180, 180, 0), ("tilt", "Inclinación", 0, 45, 0),
                                ("roll", "Roll", -180, 180, 0), ("depth", "Profundidad mm", 0, 100, 0)]:
    f = tk.Frame(tab_r)
    f.pack(fill='x', padx=10, pady=3)
    tk.Label(f, text=lbl, width=14, anchor='w').pack(side='left')
    s = tk.Scale(f, from_=lo, to=hi, orient='horizontal', length=380,
                 command=lambda _=None: debounced("rcm", apply_rcm, 250))
    s.set(init)
    s.pack(side='right')
    rcm_sliders[key] = s

rcm_enabled = tk.BooleanVar(value=False)
fk_info = tk.StringVar(value="FK sin validar: la IK está BLOQUEADA")
tk.Label(tab_r, textvariable=fk_info, fg="red").pack(pady=2)


def validate_fk():
    """Compara X,Y,Z de FK vs get_coords() del firmware en la postura actual."""
    a, c = robot.angles(), robot.coords()
    if not a or not c:
        messagebox.showerror("Error", "No se pudo leer el robot")
        return
    T = fk(a)
    err = float(np.linalg.norm(T[:3, 3] - np.array(c[:3])))
    rpy = R_to_rpy(T[:3, :3])
    print(f"FK pos={np.round(T[:3,3],1)}  firmware={np.round(c[:3],1)}  err={err:.2f} mm")
    print(f"FK rpy={np.round(rpy,1)}  firmware rpy={np.round(c[3:],1)}  (compara la convención)")
    fk_validations.append(err)
    ok = len(fk_validations) >= 3 and all(e < VALID_TOL_MM for e in fk_validations[-3:])
    rcm_enabled.set(ok)
    fk_info.set(f"Último error FK: {err:.2f} mm ({len(fk_validations)} validaciones). "
                + ("IK HABILITADA" if ok else "Necesitas 3 seguidas < %.1f mm" % VALID_TOL_MM))
    # el color solo es informativo
    for w in tab_r.winfo_children():
        if isinstance(w, tk.Label) and w.cget("textvariable") == str(fk_info):
            w.config(fg="green" if ok else "red")


tk.Button(tab_r, text="Validar FK (repite en 3+ posturas)", command=validate_fk).pack(pady=4)
step_var = tk.DoubleVar(value=3.0)
degs_var = tk.IntVar(value=20)
pf = tk.Frame(tab_r)
pf.pack(fill='x', padx=10)
tk.Label(pf, text="Paso máx. por waypoint (°)").pack(side='left')
tk.Scale(pf, from_=1, to=10, resolution=0.5, orient='horizontal', variable=step_var, length=110).pack(side='left')
tk.Label(pf, text="Vel. (°/s)").pack(side='left', padx=(10, 0))
tk.Scale(pf, from_=3, to=60, orient='horizontal', variable=degs_var, length=110).pack(side='left')


def log_row(row):
    new = not os.path.exists(LOG_PATH)
    with open(LOG_PATH, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["t", "azimuth", "tilt", "roll", "depth", "ik_ok", "max_jump_deg",
                        "sent", "cmd_q", "real_q", "real_coords", "pos_err_mm"])
        w.writerow(row)


def _log_final(params, q_cmd, jump, Td):
    rq, rc = robot.angles(), robot.coords()
    err = float(np.linalg.norm(np.array(rc[:3]) - Td[:3, 3])) if rc else ""
    log_row([time.time(), *params, 1, round(jump, 2), 1, [round(x, 2) for x in q_cmd],
             [round(x, 2) for x in rq] if rq else "", [round(x, 2) for x in rc] if rc else "",
             round(err, 2) if err != "" else ""])


def _run_rcm(gen, trocar, params, speed, max_step, deg_s):
    """Hilo de trabajo: planifica waypoints y los transmite al robot. Una orden nueva lo cancela."""
    q0 = robot.angles()
    if not q0:
        return
    qs, info = plan_to_rcm(q0, trocar, params, max_step=max_step)
    if gen != path_gen["n"]:
        return
    if qs is None:
        msg = f"Trayectoria rechazada: {info['reason']}"
        ui_q.put(lambda: status_var.set(msg))
        log_row([time.time(), *params, 0, "", 0, "", "", "", ""])
        return
    ui_q.put(lambda: status_var.set(
        f"Ejecutando {len(qs) - 1} waypoints ({'sobre pivote' if info['on_pivot'] else 'aproximación'}), "
        f"cond. jacobiano máx {info['max_cond']:.0f}"))
    if hasattr(robot.mc, "set_fresh_mode"):
        with robot.lock:
            robot.mc.set_fresh_mode(1)        # ejecutar siempre el último comando recibido
    for prev, q in zip(qs[:-1], qs[1:]):
        if gen != path_gen["n"]:
            return                             # llegó una orden nueva: abandonar esta
        robot.send_angles(list(q), speed)
        time.sleep(max(0.12, float(np.max(np.abs(q - prev))) / max(deg_s, 1)))
    if gen != path_gen["n"]:
        return
    robot.wait_arrival()
    Td, tip = rcm_flange_pose(trocar, *params)
    _log_final(params, qs[-1], float(np.max(np.abs(qs[-1] - qs[0]))), Td)
    ui_q.put(lambda: status_var.set(
        f"RCM OK  punta=({tip[0]:.0f},{tip[1]:.0f},{tip[2]:.0f})  error pivote planificado {info['pivot_err']:.2f} mm"))


def apply_rcm():
    if not rcm_enabled.get():
        status_var.set("IK bloqueada: valida la FK primero")
        return
    try:
        trocar = [float(v.get()) for v in trocar_vars]
    except ValueError:
        return
    params = [rcm_sliders[k].get() for k in ("azimuth", "tilt", "roll", "depth")]
    path_gen["n"] += 1
    threading.Thread(target=_run_rcm, daemon=True,
                     args=(path_gen["n"], trocar, params, speed_var.get(), step_var.get(), degs_var.get())).start()


# ---- Pestaña 5: rutinas
tab_rt = ttk.Frame(nb)
nb.add(tab_rt, text="Rutinas")
tk.Label(tab_rt, text="1. Libera motores  2. Mueve a mano  3. Guarda punto", justify="left").pack(pady=8)
bf = tk.Frame(tab_rt)
bf.pack(fill='x', padx=20)
lb = tk.Listbox(tab_rt, height=10)


def save_point():
    a = robot.angles()
    if a:
        routine_points.append(a)
        lb.insert(tk.END, f"P{len(routine_points)}: {[round(x, 1) for x in a]}")


def _play():
    for i, p in enumerate(routine_points):
        if routine_stop.is_set():
            break
        robot.send_angles(p, speed_var.get())
        robot.wait_arrival()          # espera real, no 2500 ms fijos
    ui_q.put(lambda: status_var.set("Rutina terminada"))


def play_routine():
    routine_stop.clear()
    status_var.set("Ejecutando rutina...")
    threading.Thread(target=_play, daemon=True).start()


def clear_routine():
    routine_points.clear()
    lb.delete(0, tk.END)


def save_json():
    path = filedialog.asksaveasfilename(defaultextension=".json")
    if path:
        with open(path, "w") as f:
            json.dump(routine_points, f)


def load_json():
    path = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
    if path:
        clear_routine()
        with open(path) as f:
            for p in json.load(f):
                routine_points.append(p)
                lb.insert(tk.END, f"P{len(routine_points)}: {[round(x, 1) for x in p]}")


for txt, col, cmd in [("Guardar punto", "lightgreen", save_point), ("Ejecutar", "gold", play_routine),
                      ("Parar", "salmon", routine_stop.set), ("Limpiar", None, clear_routine)]:
    tk.Button(bf, text=txt, bg=col, command=cmd).pack(side='left', expand=True, fill='x', padx=3)
bf2 = tk.Frame(tab_rt)
bf2.pack(fill='x', padx=20, pady=4)
tk.Button(bf2, text="Guardar JSON", command=save_json).pack(side='left', expand=True, fill='x', padx=3)
tk.Button(bf2, text="Cargar JSON", command=load_json).pack(side='left', expand=True, fill='x', padx=3)
lb.pack(fill='both', expand=True, padx=20, pady=10)

# ---- Botones globales de motores
mf = tk.Frame(root)
mf.pack(pady=8)
tk.Button(mf, text="Liberar Motores", bg="red", fg="white", font=("Arial", 12, "bold"),
          command=robot.release).pack(side='left', padx=10)
tk.Button(mf, text="Activar Motores", bg="green", fg="white", font=("Arial", 12, "bold"),
          command=robot.focus).pack(side='left', padx=10)

def poll_ui():
    while True:
        try:
            ui_q.get_nowait()()
        except queue.Empty:
            break
    root.after(100, poll_ui)


update_realtime()
poll_ui()
root.mainloop()
