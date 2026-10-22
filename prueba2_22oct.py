#!/usr/bin/env python3
"""
Simulador laparoscópico con myCobot 320 Pi  -  UN SOLO ARCHIVO (copiar y pegar en VS Code)

Instalar:   pip install numpy pymycobot
Ejecutar:   python3 mycobot_lap_sim.py                 # robot real (/dev/ttyAMA0)
            python3 mycobot_lap_sim.py --sim           # robot simulado, sin hardware
            python3 mycobot_lap_sim.py --port /dev/ttyUSB0 --baud 115200

Primer uso (en este orden):
  1. Estado        -> revisa el diagnóstico de arranque.
  2. Calibración   -> "Comparar con firmware" (NO mueve el robot); si no coincide, "Ajustar DH".
                      Hasta que la FK sea válida el RCM está bloqueado.
  3. RCM           -> "Comprobar alcance del trocar", "Ir al inicio" y botones guiados
                      (Bajar/Subir, Roll, Inclinar, Azimut). Los sliders solo cambian el objetivo.
Seguridad: PARO = tecla Esc (cancela y envía stop()). "Liberar motores" deja caer el brazo: sujétalo.

Secciones del archivo: 1) cinemática  2) robot (real/simulado)  3) interfaz.
"""
import argparse
import csv
import itertools
import json
import logging
import math
import os
import queue
import threading
import time
import tkinter as tk
from contextlib import contextmanager
from tkinter import messagebox, ttk

import numpy as np



# ============================================================================
# 1) CINEMÁTICA (DH oficial, IK, RCM, planificador, calibraciones)
# ============================================================================

# (alpha_{i-1} [deg], a_{i-1} [mm], d_i [mm], offset_i [deg])
DH_OFFICIAL = np.array([
    [0,    0,    173.9,  0],
    [90,   0,    0,     -90],
    [0,   -135,  0,      0],
    [0,   -120,  95,    -90],
    [90,   0,    87.78,  0],
    [-90,  0,    65.5,   0],
], dtype=float)

# Límites de la documentación API del 320 (send_angle). El robot puede reportar otros:
# usa Robot.read_limits() para leer los reales con get_joint_min/max_angle.
DOC_LIMITS = [(-168, 168), (-135, 135), (-145, 145), (-148, 148), (-168, 168), (-180, 180)]

ORI_WEIGHT = 50.0          # mm/rad: pondera orientación frente a posición en la IK
EULER_ORDERS = ["".join(p) for p in itertools.permutations("xyz")]


# ----------------------------------------------------------------------------
# Rotaciones y errores
# ----------------------------------------------------------------------------
def _rot(axis, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def euler_to_R(rx, ry, rz, order="xyz"):
    """Rotaciones EXTRÍNSECAS aplicadas en el orden indicado: 'xyz' => R = Rz·Ry·Rx."""
    ang = {"x": rx, "y": ry, "z": rz}
    R = np.eye(3)
    for ax in order:
        R = _rot(ax, ang[ax]) @ R
    return R


def R_to_euler(R, order="xyz", guess=None):
    """Inversa numérica de euler_to_R (grados). Útil para mostrar poses y para pruebas."""
    a = np.zeros(3) if guess is None else np.array(guess, float)
    for _ in range(60):
        e = rot_err(euler_to_R(*a, order=order), R)
        if np.linalg.norm(e) < 1e-10:
            break
        J = np.zeros((3, 3))
        for i in range(3):
            d = a.copy()
            d[i] += 1e-4
            J[:, i] = (rot_err(euler_to_R(*d, order=order), R) - e) / math.radians(1e-4)
        a = a - np.degrees(np.linalg.lstsq(J, e, rcond=None)[0])
    return [float(wrap180(v)) for v in a]


def rot_angle(Ra, Rb):
    """Ángulo (deg) de la rotación relativa entre dos orientaciones."""
    c = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def rot_err(R, Rd):
    """Vector de rotación (eje·ángulo, rad) que lleva R a Rd; válido hasta 180°."""
    Re = Rd @ R.T
    th = math.acos(max(-1.0, min(1.0, (np.trace(Re) - 1.0) / 2.0)))
    v = np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
    if th < 1e-6:
        return 0.5 * v
    if th > math.pi - 1e-3:
        M = (Re + np.eye(3)) / 2.0
        i = int(np.argmax(np.diag(M)))
        return th * M[:, i] / math.sqrt(max(M[i, i], 1e-12))
    return th / (2.0 * math.sin(th)) * v


def rodrigues(w):
    th = float(np.linalg.norm(w))
    if th < 1e-12:
        return np.eye(3)
    k = w / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * (K @ K)


def interp_pose(Ta, Tb, s):
    T = np.eye(4)
    T[:3, 3] = Ta[:3, 3] * (1 - s) + Tb[:3, 3] * s
    T[:3, :3] = rodrigues(rot_err(Ta[:3, :3], Tb[:3, :3]) * s) @ Ta[:3, :3]
    return T


def wrap180(d):
    return (d + 180.0) % 360.0 - 180.0


def pivot_distance(T, trocar):
    """Distancia (mm) del trocar a la recta del instrumento (eje Z de la brida)."""
    return float(np.linalg.norm(np.cross(np.asarray(trocar, float) - T[:3, 3], T[:3, 2])))


# ----------------------------------------------------------------------------
# Calibraciones (todas puras: reciben datos, devuelven resultados)
# ----------------------------------------------------------------------------
def identify_euler(samples, arm):
    """samples: [(q_deg, [x,y,z,rx,ry,rz])]. Devuelve (mejor_orden, {orden: error_medio_deg})."""
    errs = {}
    for order in EULER_ORDERS:
        e = [rot_angle(arm.fk(q)[:3, :3], euler_to_R(*c[3:], order=order)) for q, c in samples]
        errs[order] = float(np.mean(e))
    best = min(errs, key=errs.get)
    return best, errs


def pose_errors(samples, arm, order="xyz"):
    """Errores de posición (mm) y orientación (deg) de arm.fk frente al firmware."""
    ep, eo = [], []
    for q, c in samples:
        T = arm.fk(q)
        ep.append(float(np.linalg.norm(T[:3, 3] - np.array(c[:3]))))
        eo.append(rot_angle(T[:3, :3], euler_to_R(*c[3:], order=order)))
    return np.array(ep), np.array(eo)


# parámetros ajustables: (fila, columna) de la tabla DH => d1, a2, a3, d4, d5, d6
FIT_PARAMS = [(0, 2), (2, 1), (3, 1), (3, 2), (4, 2), (5, 2)]


def fit_dh(samples, dh0=None, iters=40, max_delta=6.0):
    """Ajusta longitudes de la tabla DH contra el FK del firmware (solo posición, no depende
    de la convención de Euler). Devuelve (dh_nuevo, rms_antes, rms_despues, deltas)."""
    dh0 = DH_OFFICIAL.copy() if dh0 is None else np.array(dh0, float)
    targets = np.array([c[:3] for _, c in samples])

    def resid(p):
        dh = dh0.copy()
        for (r, c), v in zip(FIT_PARAMS, p):
            dh[r, c] = v
        arm = Arm(dh)
        return np.concatenate([arm.fk(q)[:3, 3] - t for (q, _), t in zip(samples, targets)])

    p = np.array([dh0[r, c] for r, c in FIT_PARAMS])
    p0 = p.copy()
    r = resid(p)
    rms0 = float(np.sqrt(np.mean(r ** 2)))
    lam = 1e-3
    for _ in range(iters):
        J = np.zeros((len(r), len(p)))
        for j in range(len(p)):
            dp = p.copy()
            dp[j] += 1e-3
            J[:, j] = (resid(dp) - r) / 1e-3
        step = np.linalg.solve(J.T @ J + lam * np.eye(len(p)), -J.T @ r)
        pn = np.clip(p + step, p0 - max_delta, p0 + max_delta)
        rn = resid(pn)
        if np.sum(rn ** 2) < np.sum(r ** 2):
            p, r, lam = pn, rn, max(lam / 3, 1e-9)
            if np.linalg.norm(step) < 1e-6:
                break
        else:
            lam *= 5
    dh = dh0.copy()
    for (rr, cc), v in zip(FIT_PARAMS, p):
        dh[rr, cc] = v
    return dh, rms0, float(np.sqrt(np.mean(r ** 2))), p - p0


def pivot_calibration(flange_poses):
    """Calibración de pivote: la punta se mantiene en un punto fijo mientras se varía la
    orientación. Resuelve  R_i·t + p_i = P  por mínimos cuadrados.
    Devuelve (t_en_marco_brida, P_en_marco_base, rms_mm)."""
    A, b = [], []
    for T in flange_poses:
        A.append(np.hstack([T[:3, :3], -np.eye(3)]))
        b.append(-T[:3, 3])
    A, b = np.vstack(A), np.concatenate(b)
    x, *_ = np.linalg.lstsq(A, b, rcond=None)
    rms = float(np.sqrt(np.mean((A @ x - b) ** 2)))
    return x[:3], x[3:], rms


# ----------------------------------------------------------------------------
# Brazo: FK / IK / RCM / planificador
# ----------------------------------------------------------------------------
class Arm:
    def __init__(self, dh=None, tool_len=150.0):
        self.dh = DH_OFFICIAL.copy() if dh is None else np.array(dh, float)
        self.tool_len = float(tool_len)
        self.ik_limits = None      # None => ±180 (modo exploración); o lista [(lo,hi)]*6

    # ---- cinemática directa
    @staticmethod
    def _tf(alpha, a, theta, d):
        ca, sa = math.cos(alpha), math.sin(alpha)
        ct, st = math.cos(theta), math.sin(theta)
        return np.array([[ct, -st, 0, a],
                         [st * ca, ct * ca, -sa, -sa * d],
                         [st * sa, ct * sa, ca, ca * d],
                         [0, 0, 0, 1.0]])

    def chain(self, q):
        """Lista de 7 marcos 4x4: base y las 6 articulaciones (la última es la brida)."""
        T = np.eye(4)
        out = [T.copy()]
        for qi, (al, a, d, off) in zip(q, self.dh):
            T = T @ self._tf(math.radians(al), a, math.radians(qi + off), d)
            out.append(T.copy())
        return out

    def fk(self, q):
        return self.chain(q)[-1]

    def tip(self, q):
        T = self.fk(q)
        return T[:3, 3] + self.tool_len * T[:3, 2]

    # ---- cinemática inversa
    def _perr(self, T, Td):
        return np.hstack([Td[:3, 3] - T[:3, 3], rot_err(T[:3, :3], Td[:3, :3]) * ORI_WEIGHT])

    def _bounds(self):
        lim = self.ik_limits if self.ik_limits else [(-180, 180)] * 6
        return np.array([l[0] for l in lim], float), np.array([l[1] for l in lim], float)

    def ik(self, Td, q0, iters=300, tol_mm=0.2, tol_rad=0.002, lam=0.05):
        """IK numérica (mínimos cuadrados amortiguados), parte de q0. None si no converge."""
        lo, hi = self._bounds()
        q = np.clip(np.array(q0, float), lo, hi)
        h = 0.05
        for _ in range(iters):
            e = self._perr(self.fk(q), Td)
            if np.linalg.norm(e[:3]) < tol_mm and np.linalg.norm(e[3:]) / ORI_WEIGHT < tol_rad:
                return q
            J = np.zeros((6, 6))
            for i in range(6):
                dq = q.copy()
                dq[i] += h
                J[:, i] = (e - self._perr(self.fk(dq), Td)) / math.radians(h)
            step = np.degrees(J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(6), e))
            q = np.clip(q + np.clip(step, -5, 5), lo, hi)
        return None

    def ik_multi(self, Td, q0=None, tries=10, seed=0):
        """IK con reinicios aleatorios: sirve para saber si una pose es alcanzable."""
        lo, hi = self._bounds()
        rng = np.random.default_rng(seed)
        for k in range(tries):
            start = np.zeros(6) if (k == 0 and q0 is None) else (np.array(q0, float) if k == 0
                                                                  else rng.uniform(lo * 0.7, hi * 0.7))
            q = self.ik(Td, start, iters=200)
            if q is not None:
                return q
        return None

    def condition(self, q):
        """Número de condición del jacobiano (grande = cerca de singularidad)."""
        h, T0 = 0.05, self.fk(q)
        J = np.zeros((6, 6))
        for i in range(6):
            dq = np.array(q, float)
            dq[i] += h
            T1 = self.fk(dq)
            J[:3, i] = (T1[:3, 3] - T0[:3, 3]) / math.radians(h)
            J[3:, i] = rot_err(T0[:3, :3], T1[:3, :3]) * ORI_WEIGHT / math.radians(h)
        sv = np.linalg.svd(J, compute_uv=False)
        return float(sv[0] / max(sv[-1], 1e-9))

    # ---- RCM
    def rcm_pose(self, trocar, azimuth, tilt, roll, depth):
        """Pose de la brida para que el eje del instrumento (Z de la brida) pase por 'trocar'.
        tilt = inclinación desde la vertical (0 => instrumento hacia abajo), depth = mm de
        punta más allá del trocar. Devuelve (T_brida, punta)."""
        az, t = math.radians(azimuth), math.radians(tilt)
        u = np.array([math.sin(t) * math.cos(az), math.sin(t) * math.sin(az), -math.cos(t)])
        tip = np.asarray(trocar, float) + depth * u
        xr, yr = self._ref_axes(u)
        r = math.radians(roll)
        T = np.eye(4)
        T[:3, 0] = math.cos(r) * xr + math.sin(r) * yr
        T[:3, 1] = -math.sin(r) * xr + math.cos(r) * yr
        T[:3, 2] = u
        T[:3, 3] = tip - self.tool_len * u
        return T, tip

    @staticmethod
    def _ref_axes(u):
        ref = np.array([1.0, 0, 0]) if abs(u[0]) < 0.9 else np.array([0, 1.0, 0])
        xr = ref - ref.dot(u) * u
        xr /= np.linalg.norm(xr)
        return xr, np.cross(u, xr)

    def rcm_params(self, T, trocar, fallback_az=0.0):
        """Inversa de rcm_pose: ((az, tilt, roll, depth), distancia_al_pivote)."""
        u, f = T[:3, 2], T[:3, 3]
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, -u[2]))))
        az = math.degrees(math.atan2(u[1], u[0])) if tilt > 0.5 else fallback_az
        depth = float(np.dot(f + self.tool_len * u - np.asarray(trocar, float), u))
        xr, yr = self._ref_axes(u)
        roll = math.degrees(math.atan2(T[:3, 0].dot(yr), T[:3, 0].dot(xr)))
        return (az, tilt, roll, depth), pivot_distance(T, trocar)

    # ---- planificación
    def _chain_plan(self, q0, pose_at, trocar, max_step, max_pivot_err, max_steps):
        n, reason = 8, ""
        while n <= max_steps:
            qs, worst, ok = [q0], 0.0, True
            for k in range(1, n + 1):
                s = k / n
                q = self.ik(pose_at(s), qs[-1], iters=120)
                if q is None:
                    ok, reason = False, f"la IK no converge (tramo {s:.0%}): pose inalcanzable o singular"
                    break
                pv = pivot_distance(self.fk(q), trocar) if trocar is not None else 0.0
                if pv > max_pivot_err:
                    ok, reason = False, f"se pierde el pivote ({pv:.1f} mm) en el tramo {s:.0%}"
                    break
                jump = float(np.max(np.abs(q - qs[-1])))
                if jump > max_step:
                    ok, reason = False, f"salto articular de {jump:.0f}° en el tramo {s:.0%} (cambio de rama o singularidad)"
                    break
                worst = max(worst, pv)
                qs.append(q)
            if ok:
                sample = qs[:: max(1, len(qs) // 10)]
                return np.array(qs), {"steps": n, "pivot_err": worst, "reason": "",
                                      "max_cond": max(self.condition(q) for q in sample)}
            if reason.startswith("la IK") and n >= 16:
                break
            n *= 2
        return None, {"reason": reason}

    def plan_pose(self, q0, Td, max_step=3.0, max_steps=640):
        """Trayectoria recta en el espacio cartesiano (posición lineal + slerp) hacia Td."""
        q0 = np.array(q0, float)
        Ta = self.fk(q0)
        qs, info = self._chain_plan(q0, lambda s: interp_pose(Ta, Td, s), None, max_step, 1e9, max_steps)
        info["on_pivot"] = False
        return qs, info

    def plan_rcm(self, q0, trocar, target, max_step=3.0, max_pivot_err=1.0, max_steps=640):
        """Waypoints hacia la pose RCM target=(az,tilt,roll,depth). Si q0 ya está sobre el pivote
        interpola los PARÁMETROS (el eje no abandona el pivote); si no, aproxima en línea recta."""
        q0 = np.array(q0, float)
        Ta = self.fk(q0)
        Tb, _ = self.rcm_pose(trocar, *target)
        pa, d0 = self.rcm_params(Ta, trocar, fallback_az=target[0])
        on_pivot = d0 < 3.0
        if on_pivot:
            def pose_at(s):
                prm = (pa[0] + wrap180(target[0] - pa[0]) * s, pa[1] + (target[1] - pa[1]) * s,
                       pa[2] + wrap180(target[2] - pa[2]) * s, pa[3] + (target[3] - pa[3]) * s)
                return self.rcm_pose(trocar, *prm)[0]
        else:
            def pose_at(s):
                return interp_pose(Ta, Tb, s)
        qs, info = self._chain_plan(q0, pose_at, trocar if on_pivot else None, max_step,
                                    max_pivot_err, max_steps)
        info["on_pivot"] = on_pivot
        return qs, info

    def plan_approach(self, q0, trocar, target, max_step=3.0):
        """Primer acercamiento al pivote. Intenta la recta cartesiana; si no es viable (p. ej. desde
        el home, que es singular) usa la solución de IK más cercana y un movimiento articular
        con waypoints. OJO: en el tramo articular el eje NO está restringido al pivote."""
        q0 = np.array(q0, float)
        qs, info = self.plan_rcm(q0, trocar, target, max_step=max_step)
        if qs is not None:
            return qs, info
        Tb, _ = self.rcm_pose(trocar, *target)
        lo, hi = self._bounds()
        rng = np.random.default_rng(0)
        best = None
        for k in range(14):
            q = self.ik(Tb, q0 if k == 0 else rng.uniform(lo * 0.7, hi * 0.7), iters=200)
            if q is not None and (best is None or np.linalg.norm(q - q0) < np.linalg.norm(best - q0)):
                best = q
        if best is None:
            return None, {"reason": "la pose de inicio es inalcanzable: cambia el trocar (botón 'Comprobar alcance')",
                          "on_pivot": False}
        n = max(2, int(math.ceil(float(np.max(np.abs(best - q0))) / max_step)))
        qs = np.array([q0 + (best - q0) * s for s in np.linspace(0, 1, n + 1)])
        return qs, {"steps": n, "on_pivot": False, "pivot_err": 0.0, "reason": "", "approach": "articular",
                    "max_cond": max(self.condition(qs[0]), self.condition(qs[-1]))}

    def reach_map(self, trocar, depth, azimuths, tilts, roll=0.0, progress=None):
        """Matriz bool [tilt, azimut]: ¿es alcanzable la pose RCM? (con límites de IK)."""
        out = np.zeros((len(tilts), len(azimuths)), bool)
        total, done = out.size, 0
        for i, t in enumerate(tilts):
            for j, a in enumerate(azimuths):
                T, _ = self.rcm_pose(trocar, a, t, roll, depth)
                out[i, j] = self.ik_multi(T, tries=6) is not None
                done += 1
                if progress:
                    progress(done / total)
        return out


# ============================================================================
# 2) ROBOT (real o simulado)
# ============================================================================

log = logging.getLogger("lapsim")


def _valid(v, n=6):
    return (isinstance(v, (list, tuple)) and len(v) == n
            and all(isinstance(x, (int, float)) for x in v) and not all(x == -1 for x in v))


# ----------------------------------------------------------------------------
# Robot simulado (mismo API mínimo que MyCobot320) para desarrollar sin hardware
# ----------------------------------------------------------------------------
class SimMyCobot320:
    """El 'firmware' simulado usa su propia tabla DH ('verdadera'), que puede diferir de la
    nominal para probar las herramientas de calibración."""

    def __init__(self, dh_true=None, euler_order="xyz", deg_per_s_at_100=120.0):
        self.arm = Arm(dh_true, tool_len=0.0)
        self.order = euler_order
        self.q = np.zeros(6)
        self.target = np.zeros(6)
        self.rate = deg_per_s_at_100
        self.speed = 50
        self.fresh = 0
        self.paused = False
        self.released = False
        self.moving = False
        self._eul = [0.0, 0.0, 0.0]
        self._alive = True
        self._lock = threading.Lock()
        threading.Thread(target=self._run, daemon=True).start()

    # ---- movimiento simulado
    def _run(self):
        last = time.time()
        while self._alive:
            time.sleep(0.02)
            now = time.time()
            dt, last = now - last, now
            with self._lock:
                if self.released or self.paused:
                    self.moving = False
                    continue
                d = self.target - self.q
                dist = float(np.max(np.abs(d)))
                if dist < 1e-3:
                    self.moving = False
                    continue
                step = self.rate * self.speed / 100.0 * dt
                self.q = self.target.copy() if dist <= step else self.q + d * (step / dist)
                self.moving = True

    def close(self):
        self._alive = False

    # ---- lecturas
    def get_angles(self):
        with self._lock:
            return [round(float(x), 2) for x in self.q]

    def angles_to_coords(self, angles):
        T = self.arm.fk(angles)
        self._eul = R_to_euler(T[:3, :3], self.order, guess=self._eul)
        return [round(float(v), 2) for v in T[:3, 3]] + [round(v, 2) for v in self._eul]

    def get_coords(self):
        return self.angles_to_coords(self.get_angles())

    def get_angles_coords(self):
        return self.get_angles() + self.get_coords()

    def is_moving(self):
        return int(self.moving)

    def is_in_position(self, data, id=0):
        return int(np.max(np.abs(np.array(self.get_angles()) - np.array(data))) < 1.0)

    # ---- comandos
    def power_on(self):
        return 1

    def send_angles(self, angles, speed):
        with self._lock:
            if not self.released:
                self.target = np.clip(np.array(angles, float), [l[0] for l in DOC_LIMITS], [l[1] for l in DOC_LIMITS])
                self.speed = max(1, min(100, int(speed)))
        return 1

    def stop(self):
        with self._lock:
            self.target = self.q.copy()
        return 1

    def pause(self):
        self.paused = True
        return 1

    def resume(self):
        self.paused = False
        return 1

    def release_all_servos(self, data=None):
        self.released = True
        return 1

    def focus_all_servos(self):
        self.released = False
        with self._lock:
            self.target = self.q.copy()
        return 1

    def set_fresh_mode(self, mode):
        self.fresh = int(mode)
        return 1

    def get_fresh_mode(self):
        return self.fresh

    def set_color(self, r, g, b):
        return 1

    def set_gripper_value(self, value, speed, gripper_type=None):
        return 1

    # ---- configuración y diagnóstico
    def get_joint_min_angle(self, j):
        return float(DOC_LIMITS[j - 1][0])

    def get_joint_max_angle(self, j):
        return float(DOC_LIMITS[j - 1][1])

    def get_servo_currents(self):
        k = 150 if self.moving else 20
        return [int(220 + k * np.random.rand()) for _ in range(6)]

    def get_servo_temps(self):
        return [34 + (i % 3) for i in range(6)]

    def get_servo_voltages(self):
        return [7.4] * 6

    def get_servo_status(self):
        return [[0] * 6 for _ in range(6)]

    def read_next_error(self):
        return [0] * 6

    def get_robot_status(self):
        return [0] * 12

    def is_power_on(self):
        return 1

    def is_all_servo_enable(self):
        return 1

    def get_end_type(self):
        return 0

    def get_tool_reference(self):
        return [0] * 6

    def get_world_reference(self):
        return [0] * 6

    def get_reference_frame(self):
        return 0

    def get_movement_type(self):
        return 0

    def get_system_version(self):
        return "SIM"

    def get_basic_version(self):
        return 0.0


# ----------------------------------------------------------------------------
# Robot
# ----------------------------------------------------------------------------
class Robot:
    def __init__(self, port="/dev/ttyAMA0", baud=115200, sim=False, sim_dh=None):
        self.lock = threading.Lock()
        self.mc = None
        self.simulated = sim
        self.error = ""
        self.limits = list(DOC_LIMITS)
        self.servos_enabled = True
        self.paused = False
        self.on_stop = None            # callback: la app cancela sus trayectorias
        self.arm = None                # Arm (se asigna desde la app para comprobaciones de seguridad)
        self.safety = {"on": False, "z_min": 20.0, "max_speed": 60}
        self.state = {"t": 0.0, "angles": None, "coords": None, "health": {}, "link_ok": False}
        self._combined = True
        self._fails = 0
        self._alive = True
        try:
            if sim:
                self.mc = SimMyCobot320(dh_true=sim_dh)
            else:
                from pymycobot import MyCobot320
                self.mc = MyCobot320(port, baud)
                self.mc.power_on()
        except Exception as e:                      # noqa: BLE001
            self.error = f"{type(e).__name__}: {e}"
            log.error("No se pudo conectar: %s", self.error)
            self.mc = None
        if self.mc is not None:
            self.limits = self.read_limits()
            threading.Thread(target=self._telemetry, daemon=True).start()

    # ------------------------------------------------------------------ utilidades
    @property
    def ok(self):
        return self.mc is not None

    def _call(self, name, *args, default=None):
        """Llama a un método del robot con lock; devuelve default si falla (nunca lanza)."""
        if self.mc is None:
            return default
        try:
            with self.lock:
                return getattr(self.mc, name)(*args)
        except Exception as e:                      # noqa: BLE001
            log.warning("%s falló: %s", name, e)
            return default

    def read_limits(self):
        out = []
        for j in range(1, 7):
            lo, hi = self._call("get_joint_min_angle", j), self._call("get_joint_max_angle", j)
            ok = isinstance(lo, (int, float)) and isinstance(hi, (int, float)) and lo < hi
            out.append((float(lo), float(hi)) if ok else DOC_LIMITS[j - 1])
        return out

    def close(self):
        self._alive = False
        if hasattr(self.mc, "close"):
            self.mc.close()

    # ------------------------------------------------------------------ telemetría
    def _read_pose(self):
        if self._combined:
            v = self._call("get_angles_coords")
            if _valid(v, 12):
                return list(v[:6]), list(v[6:])
            self._combined = False          # firmware sin soporte: usar dos lecturas
        a, c = self._call("get_angles"), self._call("get_coords")
        return (list(a), list(c)) if _valid(a) and _valid(c) else (None, None)

    def _telemetry(self):
        n = 0
        while self._alive:
            a, c = self._read_pose()
            if a is not None:
                self._fails = 0
                self.state = {**self.state, "t": time.time(), "angles": a, "coords": c, "link_ok": True}
            else:
                self._fails += 1
                if self._fails >= 5:
                    self.state = {**self.state, "link_ok": False}
            if n % 10 == 0:
                self.state = {**self.state, "health": self.read_health()}
            n += 1
            time.sleep(0.2)

    def read_health(self):
        h = {}
        for key, fn in (("temps", "get_servo_temps"), ("currents", "get_servo_currents"),
                        ("volts", "get_servo_voltages"), ("errors", "read_next_error"),
                        ("status", "get_servo_status")):
            v = self._call(fn)
            if v is not None:
                h[key] = v
        return h

    def angles(self):
        return self.state["angles"]

    def coords(self):
        return self.state["coords"]

    def fresh_angles(self):
        """Lectura directa (para decisiones críticas: planificar desde la postura real)."""
        a, _ = self._read_pose()
        return a if a is not None else self.state["angles"]

    def alerts(self):
        """Lista de avisos de salud según los códigos documentados."""
        out, h = [], self.state.get("health", {})
        err = h.get("errors")
        if isinstance(err, (list, tuple)) and any(isinstance(e, int) and e not in (0, -1) for e in err):
            out.append(f"read_next_error={err}")
        st = h.get("status")
        if isinstance(st, (list, tuple)) and any(isinstance(x, (list, tuple)) and any(x) for x in st):
            out.append("servo con fallo (get_servo_status)")
        if not self.state.get("link_ok"):
            out.append("sin respuesta del robot")
        return out

    # ------------------------------------------------------------------ comandos
    def send_angles(self, angles, speed):
        """Devuelve (ok, mensaje)."""
        if not self.ok:
            return False, "sin conexión"
        if not self.servos_enabled:
            return False, "motores liberados: pulsa 'Activar motores'"
        angles = [float(a) for a in angles]
        speed = int(speed)
        if self.safety["on"]:
            angles = [max(lo, min(hi, a)) for a, (lo, hi) in zip(angles, self.limits)]
            speed = min(speed, int(self.safety["max_speed"]))
            if self.arm is not None:
                T = self.arm.fk(angles)
                if min(T[2, 3], self.arm.tip(angles)[2]) < self.safety["z_min"]:
                    return False, f"bloqueado: bajaría de Z_MIN={self.safety['z_min']:.0f} mm"
        self._call("send_angles", angles, speed)
        return True, ""

    def stop(self):
        if self.on_stop:
            self.on_stop()
        self._call("stop")

    def toggle_pause(self):
        self.paused = not self.paused
        self._call("pause" if self.paused else "resume")
        return self.paused

    def release(self, undamped=False):
        if self.on_stop:
            self.on_stop()
        self.servos_enabled = False
        self._call("release_all_servos", *((1,) if undamped else ()))

    def focus(self):
        self._call("focus_all_servos")
        time.sleep(0.3)
        self.servos_enabled = True

    def set_color(self, r, g, b):
        self._call("set_color", r, g, b)

    def gripper(self, value, speed=50):
        self._call("set_gripper_value", int(value), int(speed))

    @contextmanager
    def fresh_mode(self, on=True):
        """set_fresh_mode(1) = ejecutar siempre el último comando; se restaura al salir."""
        prev = self._call("get_fresh_mode")
        self._call("set_fresh_mode", 1 if on else 0)
        try:
            yield
        finally:
            self._call("set_fresh_mode", prev if prev in (0, 1) else 0)

    def wait_arrival(self, target=None, timeout=15.0, tol=1.0, cancel=None):
        """Espera a llegar: ángulos de telemetría cerca del objetivo o is_moving()==0 estable."""
        t0, still = time.time(), 0
        time.sleep(0.25)
        while time.time() - t0 < timeout:
            if cancel and cancel():
                return False
            a = self.state["angles"]
            if target is not None and a is not None and max(abs(x - y) for x, y in zip(a, target)) < tol:
                return True
            still = still + 1 if self._call("is_moving", default=-1) == 0 else 0
            if still >= 2 and target is None:
                return True
            time.sleep(0.1)
        return False

    # ------------------------------------------------------------------ diagnóstico
    def startup_report(self):
        """[(etiqueta, valor, estado)]  estado: ok | warn | bad"""
        rows = []

        def add(label, value, state="ok"):
            rows.append((label, str(value), state))

        add("Modo", "SIMULADO" if self.simulated else "ROBOT REAL", "warn" if self.simulated else "ok")
        add("Firmware (pico)", self._call("get_system_version", default="?"))
        add("Alimentación", {1: "encendido", 0: "apagado"}.get(self._call("is_power_on"), "?"),
            "ok" if self._call("is_power_on") == 1 else "warn")
        enabled = self._call("is_all_servo_enable")
        add("Servos conectados", {1: "todos", 0: "falta alguno"}.get(enabled, "?"), "ok" if enabled == 1 else "bad")
        rs = self._call("get_robot_status")
        add("Autodiagnóstico (get_robot_status)", rs, "ok" if not (isinstance(rs, (list, tuple)) and any(rs)) else "warn")
        er = self._call("read_next_error")
        add("Errores (read_next_error)", er, "ok" if not (isinstance(er, (list, tuple)) and any(e not in (0, -1) for e in er)) else "bad")
        et = self._call("get_end_type")
        add("Sistema final", {0: "brida (flange)", 1: "herramienta"}.get(et, et),
            "ok" if et == 0 else "warn")
        add("Referencia de herramienta", self._call("get_tool_reference"),
            "ok" if (lambda v: isinstance(v, (list, tuple)) and not any(v))(self._call("get_tool_reference")) else "warn")
        add("Marco de referencia (0=base)", self._call("get_reference_frame"),
            "ok" if self._call("get_reference_frame") == 0 else "warn")
        wr = self._call("get_world_reference")
        add("Referencia mundo", wr, "ok" if isinstance(wr, (list, tuple)) and not any(wr) else "warn")
        add("Tipo de movimiento (0=MoveJ)", self._call("get_movement_type"))
        add("Modo fresh (1=último comando)", self._call("get_fresh_mode"))
        add("Límites articulares leídos", [f"J{i+1}:{l[0]:.0f}..{l[1]:.0f}" for i, l in enumerate(self.limits)])
        return rows

    def firmware_samples(self, n=30, seed=1, progress=None):
        """Pide al controlador el FK de n posturas aleatorias SIN mover el robot
        (angles_to_coords). Devuelve [(q, coords)]."""
        rng = np.random.default_rng(seed)
        lo = np.array([l[0] for l in self.limits]) * 0.8
        hi = np.array([l[1] for l in self.limits]) * 0.8
        out = []
        for i in range(n):
            q = rng.uniform(lo, hi)
            c = self._call("angles_to_coords", [float(x) for x in q])
            if _valid(c):
                out.append((q, list(c)))
            if progress:
                progress((i + 1) / n)
        return out


# ============================================================================
# 3) INTERFAZ
# ============================================================================

#!/usr/bin/env python3
"""
Simulador laparoscópico con myCobot 320 Pi  -  aplicación principal.

    python3 app.py                 # robot real (puerto por defecto /dev/ttyAMA0)
    python3 app.py --sim           # robot simulado (sin hardware)
    python3 app.py --port /dev/ttyUSB0 --baud 115200

Flujo recomendado:
  1. Pestaña Estado        -> revisa el diagnóstico de arranque.
  2. Pestaña Calibración   -> "Comparar con firmware" (no mueve el robot). Hasta que la FK
                              propia coincida con la del robot, el RCM queda bloqueado.
  3. Pestaña RCM           -> define el trocar, "Ir al inicio" y usa los botones guiados.
"""



CFG_PATH = "lapsim_config.json"
CSV_PATH = "limites_empiricos.csv"
POS_TOL_MM, ORI_TOL_DEG = 3.0, 2.0       # tolerancia para dar por válida la FK propia
PARAM_RANGES = {"azimuth": (-180, 180), "tilt": (0, 60), "roll": (-180, 180), "depth": (-80, 120)}
PARAM_LABELS = {"azimuth": "Azimut (°)", "tilt": "Inclinación (°)", "roll": "Roll (°)", "depth": "Profundidad (mm)"}
START_PARAMS = {"azimuth": 0.0, "tilt": 0.0, "roll": 0.0, "depth": -40.0}   # 40 mm por encima del trocar
DEFAULT_POINTS = {
    "P1": [9.66, -49.48, -63.89, 5.53, 99.22, 0.26],
    "P2": [12.48, -52.82, -91.4, 69.87, 95.8, -0.08],
    "P3": [5.27, -65.47, -81.91, 81.82, 109.51, 1.14],
    "P4": [13.18, -59.58, -93.77, 61.96, 80.77, 1.14],
}
log = logging.getLogger("lapsim")


# ----------------------------------------------------------------------------
# Controlador de movimiento: UN hilo, UNA orden a la vez (gana la última), solo STOP interrumpe
# ----------------------------------------------------------------------------
class Motion:
    """Los hilos de trabajo NUNCA tocan variables de Tk: reciben una 'foto' (snap) de los parámetros
    tomada en el hilo principal en el momento de pedir el movimiento."""

    def __init__(self, app):
        self.app = app
        self.cv = threading.Condition()
        self.pending = None
        self.busy = False
        self.gen = 0
        threading.Thread(target=self._loop, daemon=True).start()

    def request(self, kind, payload):
        snap = self.app.snapshot()                      # se llama desde el hilo principal
        with self.cv:
            self.pending = (kind, payload, snap)
            self.cv.notify()

    def cancel(self):
        with self.cv:
            self.gen += 1
            self.pending = None

    def _loop(self):
        while True:
            with self.cv:
                while self.pending is None:
                    self.cv.wait()
                (kind, payload, snap), self.pending = self.pending, None
                self.busy, gen = True, self.gen
            try:
                getattr(self, "_" + kind)(payload, gen, snap)
            except Exception as e:                      # noqa: BLE001
                log.exception("error en movimiento")
                self.app.status(f"Error en el movimiento: {e}")
            finally:
                self.busy = False

    # ---- utilidades
    def _stream(self, qs, gen, snap):
        """Transmite waypoints. En modo 'fresh' el robot ejecuta siempre el último recibido."""
        r = self.app.robot
        deg_s = max(snap["deg_s"], 1)
        with r.fresh_mode(True):
            for prev, q in zip(qs[:-1], qs[1:]):
                if gen != self.gen:
                    return False
                ok, msg = r.send_angles(q, snap["speed"])
                if not ok:
                    self.app.status(msg)
                    return False
                time.sleep(max(0.12, float(np.max(np.abs(q - prev))) / deg_s))
            return r.wait_arrival(target=[float(x) for x in qs[-1]], tol=1.5, timeout=12,
                                  cancel=lambda: gen != self.gen)

    def _plan_and_go(self, planner, label, gen, snap, params=None):
        a, r = self.app, self.app.robot
        ok, msg = a.can_move(True, snap["override"])
        if not ok:
            a.status(msg)
            return
        if snap["trocar"] is None:
            a.status("Trocar inválido")
            return
        q0 = r.fresh_angles()
        if q0 is None:
            a.status("No se pudo leer la postura del robot")
            return
        t0 = time.time()
        qs, info = planner(np.array(q0, float))
        if gen != self.gen:
            return
        if qs is None:
            hint = " · usa 'Ir al inicio' primero" if (label == "RCM" and not info.get("on_pivot")) else ""
            a.status(f"{label} rechazado: {info['reason']}{hint}")
            a.log_csv(params, 0, [], None, snap)
            return
        kind = "sobre pivote" if info.get("on_pivot") else ("aproximación articular" if info.get("approach") else "trayecto recto")
        a.status(f"{label}: {len(qs) - 1} waypoints ({kind}), planificado en {1000 * (time.time() - t0):.0f} ms, "
                 f"cond. jacobiano {info['max_cond']:.0f}")
        arrived = self._stream(qs, gen, snap)
        if gen != self.gen:
            return
        a.log_csv(params, 1, qs[-1], r.fresh_angles(), snap)
        a.status(f"{label}: " + ("listo" if arrived else "terminó sin confirmar llegada (revisa Registro)"))

    # ---- órdenes
    def _rcm(self, params, gen, snap):
        p = (params["azimuth"], params["tilt"], params["roll"], params["depth"])
        self._plan_and_go(lambda q0: self.app.arm.plan_rcm(q0, snap["trocar"], p, max_step=snap["step"]),
                          "RCM", gen, snap, p)

    def _start(self, params, gen, snap):
        p = (params["azimuth"], params["tilt"], params["roll"], params["depth"])
        self._plan_and_go(lambda q0: self.app.arm.plan_approach(q0, snap["trocar"], p, max_step=snap["step"]),
                          "Inicio", gen, snap, p)

    def _pose(self, Td, gen, snap):
        self._plan_and_go(lambda q0: self.app.arm.plan_pose(q0, Td, max_step=snap["step"]), "Cartesiano", gen, snap)

    def _joints(self, angles, gen, snap):
        a = self.app
        ok, msg = a.can_move(False, snap["override"])
        if not ok:
            a.status(msg)
            return
        ok, msg = a.robot.send_angles(angles, snap["speed"])
        if not ok:
            a.status(msg)
            return
        a.robot.wait_arrival(target=list(angles), tol=1.5, cancel=lambda: gen != self.gen)

    def _seq(self, payload, gen, snap):
        a = self.app
        points, loop = payload
        ok, msg = a.can_move(False, snap["override"])
        if not ok:
            a.status(msg)
            return
        while True:
            for name, ang in points:
                if gen != self.gen:
                    return
                a.status(f"Rutina: {name}")
                a.robot.send_angles(ang, snap["speed"])
                a.robot.wait_arrival(target=list(ang), tol=1.5, cancel=lambda: gen != self.gen)
            if not loop or gen != self.gen:
                break
        a.status("Rutina terminada")


# ----------------------------------------------------------------------------
# Aplicación
# ----------------------------------------------------------------------------
class App:
    def __init__(self, root, robot, cfg):
        self.root, self.robot, self.cfg = root, robot, cfg
        self.ui_q = queue.Queue()
        dh = np.array(cfg["dh"]) if "dh" in cfg else DH_OFFICIAL
        self.arm = Arm(dh, tool_len=cfg.get("tool_len", 150.0))
        robot.arm = self.arm
        robot.on_stop = lambda: self.motion.cancel()
        self.motion = Motion(self)
        self.euler_order = cfg.get("euler_order", "xyz")
        self.validated = False
        self.cal_samples, self.pivot_poses = [], []
        self._sync = False
        self._synced_once = False
        self._live_job = None

        self.speed = tk.IntVar(value=cfg.get("speed", 30))
        self.deg_s = tk.IntVar(value=cfg.get("deg_s", 20))
        self.step_deg = tk.DoubleVar(value=cfg.get("max_step", 3.0))
        self.override = tk.BooleanVar(value=False)
        self.status_var = tk.StringVar(value="Listo" if robot.ok else f"SIN CONEXIÓN: {robot.error}")

        root.title("myCobot 320 · Simulador laparoscópico" + ("  [SIMULADO]" if robot.simulated else ""))
        root.geometry("980x760")
        root.minsize(900, 680)
        st = ttk.Style()
        st.theme_use("clam")
        st.configure("Stop.TButton", font=("Arial", 12, "bold"), foreground="white", background="#c62828")
        st.map("Stop.TButton", background=[("active", "#e53935")])
        st.configure("Big.TButton", font=("Arial", 10, "bold"), padding=6)
        self._topbar()
        self.nb = ttk.Notebook(root)
        self.nb.pack(fill="both", expand=True, padx=8, pady=4)
        self._tab_state()
        self._tab_joints()
        self._tab_coords()
        self._tab_calib()
        self._tab_rcm()
        self._tab_points()
        self._tab_log()
        ttk.Label(root, textvariable=self.status_var, relief="sunken", anchor="w", padding=4).pack(fill="x", side="bottom")
        root.bind("<Escape>", lambda e: self.emergency_stop())
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.poll()
        self.refresh()
        self.run_diagnostic()

    # ------------------------------------------------------------------ utilidades
    def post(self, fn):
        self.ui_q.put(fn)

    def status(self, text):
        log.info(text)
        self.post(lambda: self.status_var.set(text))

    def trocar(self):
        return [float(v.get()) for v in self.trocar_vars]

    def snapshot(self):
        """Parámetros leídos de Tk en el hilo principal, para entregarlos a los hilos de trabajo."""
        try:
            tro = self.trocar()
        except ValueError:
            tro = None
        return {"speed": int(self.speed.get()), "deg_s": int(self.deg_s.get()), "step": float(self.step_deg.get()),
                "trocar": tro, "override": bool(self.override.get())}

    def can_move(self, need_fk=True, override=None):
        override = self.override.get() if override is None else override
        if not self.robot.ok:
            return False, "sin conexión con el robot"
        if not self.robot.servos_enabled:
            return False, "motores liberados: pulsa 'Activar motores'"
        if need_fk and not (self.validated or override):
            return False, "FK sin validar: ve a Calibración y pulsa 'Comparar con firmware'"
        return True, ""

    def emergency_stop(self):
        self.robot.stop()
        self.robot.set_color(255, 0, 0)
        self.status("PARO: movimiento cancelado y stop() enviado al robot")

    def log_csv(self, params, ok, cmd_q, real_q, snap):
        try:
            new = not os.path.exists(CSV_PATH)
            with open(CSV_PATH, "a", newline="") as f:
                w = csv.writer(f)
                if new:
                    w.writerow(["t", "trocar", "tool_len", "azimuth", "tilt", "roll", "depth", "planificado",
                                "q_comandado", "q_real", "coords_reales", "currents_mA"])
                w.writerow([round(time.time(), 2), snap["trocar"], self.arm.tool_len,
                            *(params if params else ["", "", "", ""]), ok,
                            [round(float(x), 2) for x in cmd_q] if len(cmd_q) else "",
                            [round(x, 2) for x in real_q] if real_q else "",
                            self.robot.coords(), self.robot.state.get("health", {}).get("currents")])
        except Exception:                                # noqa: BLE001
            log.exception("no se pudo escribir el CSV")

    # ------------------------------------------------------------------ barra superior
    def _topbar(self):
        f = ttk.Frame(self.root, padding=(8, 6))
        f.pack(fill="x")
        self.lbl_conn = ttk.Label(f, text="● desconectado", foreground="#c62828", font=("Arial", 10, "bold"))
        self.lbl_conn.pack(side="left")
        ttk.Label(f, text="   Velocidad firmware:").pack(side="left")
        ttk.Scale(f, from_=1, to=100, variable=self.speed, length=130).pack(side="left", padx=4)
        ttk.Label(f, textvariable=self.speed, width=3).pack(side="left")
        self.sw_limits = tk.BooleanVar(value=False)
        ttk.Checkbutton(f, text="Límites SW", variable=self.sw_limits,
                        command=lambda: self.robot.safety.update(on=self.sw_limits.get())).pack(side="left", padx=10)
        ttk.Button(f, text="PARO  (Esc)", style="Stop.TButton", command=self.emergency_stop).pack(side="right", padx=4)
        ttk.Button(f, text="Activar motores", command=self.on_focus).pack(side="right", padx=4)
        ttk.Button(f, text="Liberar motores", command=self.on_release).pack(side="right", padx=4)
        self.btn_pause = ttk.Button(f, text="Pausa", command=self.on_pause)
        self.btn_pause.pack(side="right", padx=4)

    def on_release(self):
        if messagebox.askyesno("Liberar motores", "El brazo puede CAER por gravedad. ¿Sujetarlo y continuar?"):
            self.robot.release(undamped=self.free_var.get())
            self.status("Motores liberados: mueve el brazo a mano; 'Activar motores' para recuperar el control")

    def on_focus(self):
        self.robot.focus()
        self.robot.set_color(0, 255, 0)
        self.status("Motores activados")

    def on_pause(self):
        self.btn_pause.config(text="Reanudar" if self.robot.toggle_pause() else "Pausa")

    # ------------------------------------------------------------------ pestaña Estado
    def _tab_state(self):
        t = ttk.Frame(self.nb, padding=8)
        self.nb.add(t, text="Estado")
        left = ttk.LabelFrame(t, text="Articulaciones (° y posición dentro del rango)", padding=6)
        left.pack(side="left", fill="y")
        self.st_ang, self.st_bar = [], []
        for i in range(6):
            r = ttk.Frame(left)
            r.pack(fill="x", pady=3)
            ttk.Label(r, text=f"J{i + 1}", width=3).pack(side="left")
            l = ttk.Label(r, text="   --  ", width=9, font=("Courier", 11))
            l.pack(side="left")
            b = ttk.Progressbar(r, length=150, maximum=100)
            b.pack(side="left", padx=4)
            self.st_ang.append(l)
            self.st_bar.append(b)
        cf = ttk.LabelFrame(left, text="Coordenadas (brida, marco base)", padding=6)
        cf.pack(fill="x", pady=8)
        self.st_coord = []
        for n in ("X", "Y", "Z", "RX", "RY", "RZ"):
            l = ttk.Label(cf, text=f"{n}: --", font=("Courier", 11))
            l.pack(anchor="w")
            self.st_coord.append((n, l))
        self.free_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(left, text="Liberar sin amortiguación (más libre)", variable=self.free_var).pack(anchor="w", pady=4)
        right = ttk.Frame(t)
        right.pack(side="left", fill="both", expand=True, padx=10)
        ttk.Label(right, text="Diagnóstico de arranque", font=("Arial", 10, "bold")).pack(anchor="w")
        self.tree = ttk.Treeview(right, columns=("v", "s"), height=13)
        self.tree.heading("#0", text="Prueba")
        self.tree.heading("v", text="Valor")
        self.tree.heading("s", text="")
        self.tree.column("#0", width=230)
        self.tree.column("v", width=270)
        self.tree.column("s", width=40, anchor="center")
        self.tree.pack(fill="both", expand=True)
        for tag, col in (("ok", "#1b5e20"), ("warn", "#e65100"), ("bad", "#b71c1c")):
            self.tree.tag_configure(tag, foreground=col)
        ttk.Button(right, text="Repetir diagnóstico", command=self.run_diagnostic).pack(anchor="e", pady=4)
        self.lbl_health = ttk.Label(right, text="", justify="left", font=("Courier", 9))
        self.lbl_health.pack(anchor="w")

    def run_diagnostic(self):
        def work():
            rows = self.robot.startup_report() if self.robot.ok else [("Conexión", self.robot.error or "sin robot", "bad")]
            self.post(lambda: self._fill_tree(rows))
        threading.Thread(target=work, daemon=True).start()

    def _fill_tree(self, rows):
        self.tree.delete(*self.tree.get_children())
        for label, val, state in rows:
            self.tree.insert("", "end", text=label, values=(val, {"ok": "✔", "warn": "!", "bad": "✖"}[state]), tags=(state,))

    # ------------------------------------------------------------------ pestaña Articulaciones
    def _tab_joints(self):
        t = ttk.Frame(self.nb, padding=10)
        self.nb.add(t, text="Articulaciones")
        self.j_vars, self.j_scales = [], []
        for i in range(6):
            r = ttk.Frame(t)
            r.pack(fill="x", pady=5)
            ttk.Label(r, text=f"J{i + 1}", width=4).pack(side="left")
            v = tk.DoubleVar(value=0.0)
            lo, hi = self.robot.limits[i]
            s = ttk.Scale(r, from_=lo, to=hi, variable=v, length=520, command=lambda _=None: self._joint_changed())
            s.pack(side="left", padx=6)
            ttk.Spinbox(r, from_=lo, to=hi, textvariable=v, width=8, increment=1.0,
                        command=self._joint_changed).pack(side="left")
            ttk.Label(r, text=f"[{lo:.0f}, {hi:.0f}]", foreground="gray").pack(side="left", padx=6)
            self.j_vars.append(v)
            self.j_scales.append(s)
        r = ttk.Frame(t)
        r.pack(fill="x", pady=12)
        self.live_j = tk.BooleanVar(value=False)
        ttk.Checkbutton(r, text="Envío en vivo al mover", variable=self.live_j).pack(side="left")
        ttk.Button(r, text="Enviar", style="Big.TButton", command=self.send_joints).pack(side="left", padx=10)
        ttk.Button(r, text="Copiar postura actual", command=self.sync_joint_sliders).pack(side="left")
        ttk.Button(r, text="Origen (0°)", command=lambda: self.go_joints([0] * 6)).pack(side="left", padx=10)
        ttk.Label(t, text="Los sliders arrancan sincronizados con el robot y NO envían nada hasta que pulses 'Enviar' "
                          "(o actives el envío en vivo).", foreground="gray").pack(anchor="w")

    def _joint_changed(self):
        if self._sync or not self.live_j.get():
            return
        if self._live_job:
            self.root.after_cancel(self._live_job)
        self._live_job = self.root.after(350, self.send_joints)

    def sync_joint_sliders(self):
        a = self.robot.angles()
        if a:
            self._sync = True
            for v, x in zip(self.j_vars, a):
                v.set(round(x, 2))
            self._sync = False

    def send_joints(self):
        self.motion.request("joints", [v.get() for v in self.j_vars])

    def go_joints(self, angles):
        self._sync = True
        for v, x in zip(self.j_vars, angles):
            v.set(x)
        self._sync = False
        self.motion.request("joints", list(angles))

    # ------------------------------------------------------------------ pestaña Coordenadas
    def _tab_coords(self):
        t = ttk.Frame(self.nb, padding=10)
        self.nb.add(t, text="Coordenadas")
        ttk.Label(t, text="Destino de la brida (marco base)", font=("Arial", 10, "bold")).pack(anchor="w")
        g = ttk.Frame(t)
        g.pack(anchor="w", pady=6)
        self.c_vars = []
        for i, n in enumerate(("X", "Y", "Z", "RX", "RY", "RZ")):
            ttk.Label(g, text=n, width=4).grid(row=i // 3, column=(i % 3) * 2, pady=4)
            v = tk.StringVar(value="200" if n == "Z" else "0")
            ttk.Entry(g, textvariable=v, width=9, justify="center").grid(row=i // 3, column=(i % 3) * 2 + 1, padx=6)
            self.c_vars.append(v)
        ttk.Button(t, text="Usar posición actual", command=self.coords_from_current).pack(anchor="w", pady=4)
        b = ttk.Frame(t)
        b.pack(anchor="w", pady=8)
        ttk.Button(b, text="Firmware MoveJ", command=lambda: self.fw_coords(0)).pack(side="left", padx=4)
        ttk.Button(b, text="Firmware MoveL (recta)", command=lambda: self.fw_coords(1)).pack(side="left", padx=4)
        ttk.Button(b, text="Planificador propio (waypoints)", style="Big.TButton",
                   command=self.own_coords).pack(side="left", padx=10)
        ttk.Label(t, text="El planificador propio hace una recta cartesiana con IK continua y rechaza los "
                          "trayectos que cruzan singularidades. Requiere FK validada.", foreground="gray",
                  wraplength=760).pack(anchor="w", pady=6)

    def _read_coords(self):
        try:
            return [float(v.get()) for v in self.c_vars]
        except ValueError:
            messagebox.showerror("Error", "Ingresa solo números")
            return None

    def coords_from_current(self):
        c = self.robot.coords()
        if c:
            for v, x in zip(self.c_vars, c):
                v.set(f"{x:.2f}")

    def fw_coords(self, mode):
        c = self._read_coords()
        ok, msg = self.can_move(need_fk=False)
        if c is None or not ok:
            self.status(msg or "valores inválidos")
            return
        self.robot._call("send_coords", c, int(self.speed.get()), mode)

    def own_coords(self):
        c = self._read_coords()
        if c is None:
            return
        T = np.eye(4)
        T[:3, :3] = euler_to_R(*c[3:], order=self.euler_order)
        T[:3, 3] = c[:3]
        self.motion.request("pose", T)

    # ------------------------------------------------------------------ pestaña Calibración
    def _tab_calib(self):
        t = ttk.Frame(self.nb, padding=10)
        self.nb.add(t, text="Calibración")
        a = ttk.LabelFrame(t, text="1. Validar la cinemática contra el firmware (no mueve el robot)", padding=8)
        a.pack(fill="x")
        r = ttk.Frame(a)
        r.pack(fill="x")
        ttk.Button(r, text="Comparar con firmware (30 posturas)", style="Big.TButton", command=self.cal_firmware).pack(side="left")
        ttk.Button(r, text="Añadir postura actual", command=self.cal_add_current).pack(side="left", padx=6)
        ttk.Button(r, text="Ajustar DH", command=self.cal_fit).pack(side="left")
        ttk.Button(r, text="Restaurar DH oficial", command=self.cal_reset).pack(side="left", padx=6)
        ttk.Checkbutton(r, text="Omitir validación (bajo mi riesgo)", variable=self.override).pack(side="right")
        self.cal_text = tk.Text(a, height=9, width=100, font=("Courier", 9))
        self.cal_text.pack(fill="x", pady=6)
        p = ttk.LabelFrame(t, text="2. Calibración de pivote: define punta del instrumento y trocar con datos reales", padding=8)
        p.pack(fill="x", pady=10)
        ttk.Label(p, text="Libera los motores, apoya la punta del instrumento en el punto que será el trocar y, SIN despegarla, "
                          "cambia la orientación del brazo. Registra 8 o más muestras con orientaciones bien distintas.",
                  wraplength=860, foreground="gray").pack(anchor="w")
        r2 = ttk.Frame(p)
        r2.pack(fill="x", pady=6)
        ttk.Button(r2, text="Registrar muestra", command=self.pivot_add).pack(side="left")
        ttk.Button(r2, text="Calcular", command=self.pivot_solve).pack(side="left", padx=6)
        ttk.Button(r2, text="Aplicar (trocar y longitud de herramienta)", command=self.pivot_apply).pack(side="left")
        ttk.Button(r2, text="Borrar muestras", command=self.pivot_clear).pack(side="left", padx=6)
        self.lbl_pivot = ttk.Label(p, text="0 muestras", font=("Courier", 9), justify="left")
        self.lbl_pivot.pack(anchor="w")
        self._pivot_result = None

    def cal_write(self, text):
        def f():
            self.cal_text.delete("1.0", "end")
            self.cal_text.insert("end", text)
        self.post(f)

    def cal_firmware(self):
        def work():
            if not self.robot.ok:
                self.cal_write("Sin conexión.")
                return
            self.cal_write("Pidiendo al controlador la FK de 30 posturas aleatorias (angles_to_coords)...\n")
            smp = self.robot.firmware_samples(30)
            if len(smp) < 6:
                self.cal_write("El robot no respondió a angles_to_coords (¿versión de pymycobot/firmware antigua?).\n"
                               "Alternativa: mueve el brazo a 6+ posturas distintas y pulsa 'Añadir postura actual'.")
                return
            self.cal_samples = smp
            self._analyze()
        threading.Thread(target=work, daemon=True).start()

    def cal_add_current(self):
        a, c = self.robot.angles(), self.robot.coords()
        if not a or not c:
            self.status("Sin lectura del robot")
            return
        self.cal_samples.append((np.array(a, float), list(c)))
        if len(self.cal_samples) >= 6:
            threading.Thread(target=self._analyze, daemon=True).start()
        else:
            self.cal_write(f"{len(self.cal_samples)} muestras; faltan {6 - len(self.cal_samples)} posturas distintas.")

    def _analyze(self):
        smp = self.cal_samples
        order, errs = identify_euler(smp, self.arm)
        ep, eo = pose_errors(smp, self.arm, order)
        ok = ep.max() < POS_TOL_MM and eo.max() < ORI_TOL_DEG
        self.euler_order = order
        self.validated = bool(ok)
        txt = (f"Muestras: {len(smp)}\n"
               f"Error de posición (mm):    medio {ep.mean():.2f}   máx {ep.max():.2f}   (tolerancia {POS_TOL_MM})\n"
               f"Error de orientación (°):  medio {eo.mean():.2f}   máx {eo.max():.2f}   (tolerancia {ORI_TOL_DEG})\n"
               f"Convención Euler de get_coords() más probable: '{order}' (error medio {errs[order]:.2f}°)\n\n"
               + ("FK VÁLIDA: el RCM queda habilitado.\n" if ok else
                  "FK NO válida todavía. Prueba 'Ajustar DH' (corrige longitudes de la tabla contra el firmware).\n"))
        self.cal_write(txt)
        self.status("FK validada" if ok else "FK sin validar")

    def cal_fit(self):
        if len(self.cal_samples) < 8:
            messagebox.showinfo("Ajuste", "Primero compara con el firmware (se necesitan 8+ muestras).")
            return

        def work():
            self.cal_write("Ajustando longitudes DH (puede tardar unos segundos)...")
            dh, r0, r1, delta = fit_dh(self.cal_samples, self.arm.dh, iters=25)
            names = ["d1", "a2", "a3", "d4", "d5", "d6"]
            self.arm.dh = dh
            self.cfg["dh"] = dh.tolist()
            self._analyze()
            self.post(lambda: self.cal_text.insert(
                "end", f"\nAjuste aplicado: RMS posición {r0:.2f} -> {r1:.3f} mm\n"
                       + "  ".join(f"{n}{d:+.2f}" for n, d in zip(names, delta)) + " mm\n"))
        threading.Thread(target=work, daemon=True).start()

    def cal_reset(self):
        self.arm.dh = DH_OFFICIAL.copy()
        self.cfg.pop("dh", None)
        self.validated = False
        self.cal_write("Tabla DH oficial restaurada. Vuelve a comparar con el firmware.")

    def pivot_add(self):
        a = self.robot.angles()
        if not a:
            return
        self.pivot_poses.append(self.arm.fk(a))
        self.lbl_pivot.config(text=f"{len(self.pivot_poses)} muestras")

    def pivot_solve(self):
        if len(self.pivot_poses) < 6:
            messagebox.showinfo("Pivote", "Registra al menos 6 muestras (mejor 10+).")
            return
        t, P, rms = pivot_calibration(self.pivot_poses)
        self._pivot_result = (t, P, rms)
        lateral = float(np.hypot(t[0], t[1]))
        self.lbl_pivot.config(text=f"{len(self.pivot_poses)} muestras\npunta (marco brida): "
                                   f"({t[0]:.1f}, {t[1]:.1f}, {t[2]:.1f}) mm   longitud {np.linalg.norm(t):.1f} mm\n"
                                   f"pivote (marco base): ({P[0]:.1f}, {P[1]:.1f}, {P[2]:.1f}) mm   residuo RMS {rms:.2f} mm\n"
                                   + ("AVISO: el instrumento no está alineado con el eje Z de la brida (desvío "
                                      f"lateral {lateral:.1f} mm)." if lateral > 3 else "Instrumento alineado con el eje Z de la brida."))

    def pivot_apply(self):
        if not self._pivot_result:
            return
        t, P, _ = self._pivot_result
        self.arm.tool_len = float(np.linalg.norm(t))
        self.tool_var.set(f"{self.arm.tool_len:.1f}")
        for v, x in zip(self.trocar_vars, P):
            v.set(f"{x:.1f}")
        self.status("Longitud de herramienta y trocar actualizados desde la calibración de pivote")

    def pivot_clear(self):
        self.pivot_poses.clear()
        self._pivot_result = None
        self.lbl_pivot.config(text="0 muestras")

    # ------------------------------------------------------------------ pestaña RCM
    def _tab_rcm(self):
        t = ttk.Frame(self.nb, padding=8)
        self.nb.add(t, text="RCM Laparoscopía")
        left = ttk.Frame(t)
        left.pack(side="left", fill="y")
        self.lbl_gate = ttk.Label(left, text="", font=("Arial", 9, "bold"))
        self.lbl_gate.pack(anchor="w")
        tr = ttk.LabelFrame(left, text="Trocar (punto fijo, mm, marco base)", padding=6)
        tr.pack(fill="x", pady=6)
        self.trocar_vars = []
        row = ttk.Frame(tr)
        row.pack(fill="x")
        for n, dv in zip("XYZ", self.cfg.get("trocar", [200, 0, 100])):
            ttk.Label(row, text=n).pack(side="left")
            v = tk.StringVar(value=str(dv))
            ttk.Entry(row, textvariable=v, width=7, justify="center").pack(side="left", padx=4)
            self.trocar_vars.append(v)
        row2 = ttk.Frame(tr)
        row2.pack(fill="x", pady=4)
        ttk.Label(row2, text="Longitud herramienta (mm)").pack(side="left")
        self.tool_var = tk.StringVar(value=str(self.arm.tool_len))
        e = ttk.Entry(row2, textvariable=self.tool_var, width=7, justify="center")
        e.pack(side="left", padx=4)
        e.bind("<FocusOut>", lambda ev: self._apply_tool())
        e.bind("<Return>", lambda ev: self._apply_tool())
        ttk.Button(tr, text="Trocar = punta actual", command=self.trocar_from_tip).pack(anchor="w")
        ttk.Button(tr, text="Comprobar alcance del trocar", command=self.check_reach).pack(anchor="w", pady=2)

        gd = ttk.LabelFrame(left, text="Movimiento guiado (sobre el pivote)", padding=6)
        gd.pack(fill="x", pady=6)
        ttk.Button(gd, text="Ir al inicio (vertical, 40 mm sobre el trocar)", command=self.go_start).pack(fill="x", pady=2)
        self.steps = {k: tk.DoubleVar(value=v) for k, v in (("depth", 5.0), ("roll", 5.0), ("tilt", 3.0), ("azimuth", 5.0))}
        for key, up_txt, down_txt, minus_sign in (("depth", "▼ Bajar", "▲ Subir", -1), ("roll", "⟳ Roll +", "⟲ Roll −", -1),
                                                  ("tilt", "Inclinar +", "Inclinar −", -1), ("azimuth", "Azimut +", "Azimut −", -1)):
            r = ttk.Frame(gd)
            r.pack(fill="x", pady=2)
            ttk.Button(r, text=down_txt, width=11, command=lambda k=key: self.nudge(k, -self.steps[k].get())).pack(side="left")
            ttk.Button(r, text=up_txt, width=11, command=lambda k=key: self.nudge(k, +self.steps[k].get())).pack(side="left", padx=3)
            ttk.Label(r, text="paso").pack(side="left")
            ttk.Spinbox(r, from_=0.5, to=30, increment=0.5, textvariable=self.steps[key], width=5).pack(side="left", padx=2)
        ttk.Button(gd, text="Leer parámetros de la pose actual", command=self.read_params).pack(fill="x", pady=(6, 0))

        self.rcm_vars, self.rcm_lbl = {}, {}
        sl = ttk.LabelFrame(left, text="Objetivo (los sliders NO mueven el robot)", padding=6)
        sl.pack(fill="x", pady=6)
        for k in ("azimuth", "tilt", "roll", "depth"):
            r = ttk.Frame(sl)
            r.pack(fill="x")
            ttk.Label(r, text=PARAM_LABELS[k], width=16).pack(side="left")
            v = tk.DoubleVar(value=START_PARAMS[k])
            lo, hi = PARAM_RANGES[k]
            ttk.Scale(r, from_=lo, to=hi, variable=v, length=150,
                      command=lambda _=None, kk=k: self._rcm_slider(kk)).pack(side="left")
            l = ttk.Label(r, text="0", width=6)
            l.pack(side="left")
            self.rcm_vars[k], self.rcm_lbl[k] = v, l
        r = ttk.Frame(sl)
        r.pack(fill="x", pady=4)
        ttk.Button(r, text="Ejecutar objetivo", style="Big.TButton", command=self.run_rcm).pack(side="left")
        self.live_rcm = tk.BooleanVar(value=False)
        ttk.Checkbutton(r, text="Seguimiento en vivo", variable=self.live_rcm).pack(side="left", padx=8)
        opt = ttk.Frame(left)
        opt.pack(fill="x")
        ttk.Label(opt, text="Paso máx/waypoint (°)").pack(side="left")
        ttk.Spinbox(opt, from_=1, to=10, increment=0.5, textvariable=self.step_deg, width=4).pack(side="left", padx=3)
        ttk.Label(opt, text="Vel (°/s)").pack(side="left", padx=(8, 0))
        ttk.Spinbox(opt, from_=3, to=60, increment=1, textvariable=self.deg_s, width=4).pack(side="left", padx=3)
        self.ik_lim = tk.BooleanVar(value=False)
        ttk.Checkbutton(left, text="Limitar IK a los límites articulares del robot", variable=self.ik_lim,
                        command=lambda: setattr(self.arm, "ik_limits", self.robot.limits if self.ik_lim.get() else None)).pack(anchor="w", pady=4)

        right = ttk.Frame(t)
        right.pack(side="left", fill="both", expand=True, padx=8)
        self.cv_w, self.cv_h = 520, 330
        self.canvas = tk.Canvas(right, width=self.cv_w, height=self.cv_h, bg="white", highlightthickness=1,
                                highlightbackground="#999")
        self.canvas.pack()
        ttk.Label(right, text="Azul: brazo · rojo: instrumento · verde: trocar · naranja: objetivo", foreground="gray").pack()
        self.lbl_pose = ttk.Label(right, text="", font=("Courier", 9), justify="left")
        self.lbl_pose.pack(anchor="w", pady=6)

    def _apply_tool(self):
        try:
            self.arm.tool_len = float(self.tool_var.get())
        except ValueError:
            self.tool_var.set(str(self.arm.tool_len))

    def trocar_from_tip(self):
        a = self.robot.angles()
        if not a:
            return
        self._apply_tool()
        for v, x in zip(self.trocar_vars, self.arm.tip(a)):
            v.set(f"{x:.1f}")
        self.status("Trocar fijado en la punta actual (calculada con la FK propia y la longitud de herramienta)")

    def check_reach(self):
        """Prueba 23 poses RCM alrededor del trocar (sin mover el robot) y dice si es un buen sitio."""
        self._apply_tool()
        try:
            tro = np.array(self.trocar(), float)
        except ValueError:
            self.status("Trocar inválido")
            return
        poses = [(0, 0, 0, -40), (0, 0, 0, 0), (0, 0, 0, 40)]
        poses += [(az, tl, 0, d) for az in (0, 90, 180, 270) for tl in (10, 20) for d in (0, 30)]
        poses += [(0, 0, -30, 0), (0, 0, 30, 0)]

        def work():
            self.status("Comprobando alcance del trocar (unos segundos)...")
            ok, start_ok = 0, False
            for i, p in enumerate(poses):
                if self.arm.ik_multi(self.arm.rcm_pose(tro, *p)[0], tries=5) is not None:
                    ok += 1
                    start_ok = start_ok or i == 0
            self.status(f"Alcance del trocar: {ok}/{len(poses)} poses de prueba ({ok / len(poses):.0%}); "
                        f"pose de inicio {'alcanzable' if start_ok else 'NO alcanzable: mueve el trocar'}")
        threading.Thread(target=work, daemon=True).start()

    def params(self):
        return {k: v.get() for k, v in self.rcm_vars.items()}

    def _rcm_slider(self, k):
        self.rcm_lbl[k].config(text=f"{self.rcm_vars[k].get():.0f}")
        if self.live_rcm.get():
            if self._live_job:
                self.root.after_cancel(self._live_job)
            self._live_job = self.root.after(450, self.run_rcm)

    def run_rcm(self):
        self._apply_tool()
        try:
            self.trocar()
        except ValueError:
            self.status("Trocar inválido")
            return
        self.motion.request("rcm", self.params())

    def nudge(self, key, delta):
        lo, hi = PARAM_RANGES[key]
        v = self.rcm_vars[key].get() + delta
        if key in ("azimuth", "roll"):
            v = (v + 180) % 360 - 180
        self.rcm_vars[key].set(max(lo, min(hi, v)))
        self._rcm_slider(key)
        self.run_rcm()

    def go_start(self):
        for k, v in START_PARAMS.items():
            self.rcm_vars[k].set(v)
            self.rcm_lbl[k].config(text=f"{v:.0f}")
        self._apply_tool()
        self.motion.request("start", dict(START_PARAMS))

    def read_params(self):
        a = self.robot.angles()
        if not a:
            return
        self._apply_tool()
        prm, d = self.arm.rcm_params(self.arm.fk(a), self.trocar(), fallback_az=self.rcm_vars["azimuth"].get())
        if d > 3.0:
            self.status(f"La pose actual NO está sobre el pivote (a {d:.1f} mm). Usa 'Ir al inicio'.")
            return
        for k, x in zip(("azimuth", "tilt", "roll", "depth"), prm):
            lo, hi = PARAM_RANGES[k]
            self.rcm_vars[k].set(max(lo, min(hi, x)))
            self.rcm_lbl[k].config(text=f"{x:.0f}")

    # ------------------------------------------------------------------ vistas 2D
    def draw_views(self):
        c = self.canvas
        c.delete("all")
        q = self.robot.angles() or [0] * 6
        pts = [T[:3, 3] for T in self.arm.chain(q)]
        tip = self.arm.tip(q)
        try:
            tro = np.array(self.trocar(), float)
            prm = self.params()
            Tt, tip_t = self.arm.rcm_pose(tro, prm["azimuth"], prm["tilt"], prm["roll"], prm["depth"])
        except ValueError:
            tro, Tt, tip_t = None, None, None
        s = 0.30
        for title, ox, oy, ia, ib in (("Vista superior (X-Y)", 135, 175, 0, 1), ("Vista lateral (X-Z)", 395, 300, 0, 2)):
            def P(p, ox=ox, oy=oy, ia=ia, ib=ib):
                return ox + p[ia] * s, oy - p[ib] * s
            c.create_text(ox, 12, text=title, fill="#555")
            c.create_line(ox - 120, oy, ox + 120, oy, fill="#ddd")
            c.create_line(ox, oy - (150 if ib == 1 else 0), ox, oy + (150 if ib == 1 else 0) if ib == 1 else oy, fill="#ddd")
            c.create_oval(*(np.array(P(pts[0])) - 4), *(np.array(P(pts[0])) + 4), fill="#444")
            flat = [v for p in pts for v in P(p)]
            c.create_line(*flat, width=4, fill="#1565c0", capstyle="round", joinstyle="round")
            for p in pts[1:]:
                x, y = P(p)
                c.create_oval(x - 3, y - 3, x + 3, y + 3, fill="white", outline="#1565c0")
            c.create_line(*P(pts[-1]), *P(tip), width=3, fill="#d32f2f")
            if tro is not None:
                x, y = P(tro)
                c.create_oval(x - 6, y - 6, x + 6, y + 6, outline="#2e7d32", width=3)
                x2, y2 = P(Tt[:3, 3])
                x3, y3 = P(tip_t)
                c.create_line(x2, y2, x3, y3, fill="#ef6c00", dash=(4, 3), width=2)

    # ------------------------------------------------------------------ pestaña Puntos y rutinas
    def _tab_points(self):
        t = ttk.Frame(self.nb, padding=8)
        self.nb.add(t, text="Puntos y rutinas")
        self.points = self.cfg.get("points", dict(DEFAULT_POINTS))
        left = ttk.LabelFrame(t, text="Puntos guardados (ángulos articulares)", padding=6)
        left.pack(side="left", fill="both", expand=True)
        self.lb = tk.Listbox(left, height=14, font=("Courier", 9), exportselection=False)
        self.lb.pack(fill="both", expand=True)
        b = ttk.Frame(left)
        b.pack(fill="x", pady=6)
        ttk.Button(b, text="Ir", command=self.pt_go).pack(side="left")
        ttk.Button(b, text="Guardar postura actual", command=self.pt_save).pack(side="left", padx=4)
        ttk.Button(b, text="Borrar", command=self.pt_del).pack(side="left")
        r = ttk.LabelFrame(t, text="Rutina (todos los puntos, en orden)", padding=6)
        r.pack(side="left", fill="y", padx=8)
        self.loop = tk.BooleanVar(value=False)
        ttk.Checkbutton(r, text="Repetir en bucle", variable=self.loop).pack(anchor="w")
        ttk.Button(r, text="Ejecutar rutina", style="Big.TButton", command=self.pt_play).pack(fill="x", pady=4)
        ttk.Label(r, text="Para detenerla: PARO (Esc)", foreground="gray").pack()
        self.pt_refresh()

    def pt_refresh(self):
        self.lb.delete(0, "end")
        for n, a in self.points.items():
            self.lb.insert("end", f"{n:8s} {[round(x, 1) for x in a]}")

    def _pt_sel(self):
        s = self.lb.curselection()
        return list(self.points.items())[s[0]] if s else None

    def pt_go(self):
        p = self._pt_sel()
        if p:
            self.go_joints(p[1])

    def pt_save(self):
        a = self.robot.angles()
        if a:
            self.points[f"P{len(self.points) + 1}"] = [round(x, 2) for x in a]
            self.pt_refresh()

    def pt_del(self):
        p = self._pt_sel()
        if p:
            self.points.pop(p[0])
            self.pt_refresh()

    def pt_play(self):
        self.motion.request("seq", (list(self.points.items()), self.loop.get()))

    # ------------------------------------------------------------------ pestaña Registro
    def _tab_log(self):
        t = ttk.Frame(self.nb, padding=8)
        self.nb.add(t, text="Registro")
        self.log_text = tk.Text(t, font=("Courier", 9), wrap="word")
        self.log_text.pack(fill="both", expand=True)
        app = self

        class H(logging.Handler):
            def emit(self, rec):
                msg = f"{time.strftime('%H:%M:%S')} {rec.levelname[:4]} {rec.getMessage()}\n"
                app.post(lambda: (app.log_text.insert("end", msg), app.log_text.see("end")))
        logging.getLogger("lapsim").addHandler(H())
        ttk.Label(t, text=f"Los intentos RCM se guardan en {CSV_PATH} (ángulos comandados vs reales, coordenadas y corrientes).",
                  foreground="gray").pack(anchor="w")

    # ------------------------------------------------------------------ bucles de interfaz
    def poll(self):
        while True:
            try:
                self.ui_q.get_nowait()()
            except queue.Empty:
                break
        self.root.after(100, self.poll)

    def refresh(self):
        st = self.robot.state
        if self.robot.ok:
            alerts = self.robot.alerts()
            if st["link_ok"] and not alerts:
                self.lbl_conn.config(text="● conectado" + (" (sim)" if self.robot.simulated else ""), foreground="#2e7d32")
            else:
                self.lbl_conn.config(text="● " + ("; ".join(alerts) or "sin datos"), foreground="#e65100")
        a, c = st["angles"], st["coords"]
        if a:
            if not self._synced_once:
                self._synced_once = True
                self.sync_joint_sliders()
            for i, (lbl, bar) in enumerate(zip(self.st_ang, self.st_bar)):
                lo, hi = self.robot.limits[i]
                lbl.config(text=f"{a[i]:8.2f}")
                bar["value"] = 100 * (a[i] - lo) / (hi - lo)
        if c:
            for (n, l), x in zip(self.st_coord, c):
                l.config(text=f"{n:>2}: {x:9.2f}")
        h = st.get("health", {})
        self.lbl_health.config(text=(f"temp °C : {h.get('temps')}\ncorriente mA: {h.get('currents')}\n"
                                     f"voltaje V: {h.get('volts')}") if h else "")
        gate = ("FK validada ✔ (Euler '%s')" % self.euler_order) if self.validated else (
            "FK omitida (bajo tu riesgo)" if self.override.get() else "FK SIN VALIDAR → pestaña Calibración")
        self.lbl_gate.config(text=gate, foreground="#2e7d32" if self.validated else "#c62828")
        if a:
            try:
                self.draw_views()
                prm, d = self.arm.rcm_params(self.arm.fk(a), self.trocar(), fallback_az=0)
                self.lbl_pose.config(text=f"Punta: ({self.arm.tip(a)[0]:.0f}, {self.arm.tip(a)[1]:.0f}, {self.arm.tip(a)[2]:.0f}) mm\n"
                                          f"Eje a {d:.1f} mm del trocar" + ("  → sobre pivote" if d < 3 else "  → fuera del pivote")
                                          + f"\n{'EN MOVIMIENTO' if self.motion.busy else 'quieto'}")
            except ValueError:
                pass
        self.root.after(250, self.refresh)

    def on_close(self):
        try:
            self.cfg.update(trocar=[float(v.get()) for v in self.trocar_vars], tool_len=self.arm.tool_len,
                            speed=self.speed.get(), deg_s=self.deg_s.get(), max_step=self.step_deg.get(),
                            euler_order=self.euler_order, points=self.points)
            with open(CFG_PATH, "w") as f:
                json.dump(self.cfg, f, indent=1)
        except Exception:                                # noqa: BLE001
            log.exception("no se pudo guardar la configuración")
        self.robot.close()
        self.root.destroy()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--sim", action="store_true", help="robot simulado (sin hardware)")
    ap.add_argument("--sim-dh-error", action="store_true", help="el simulador tiene d5 +1 mm para probar el ajuste de DH")
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, filename="lapsim.log", format="%(asctime)s %(levelname)s %(message)s")
    try:
        cfg = json.load(open(CFG_PATH))
    except Exception:                                    # noqa: BLE001
        cfg = {}
    sim_dh = None
    if args.sim_dh_error:
        sim_dh = DH_OFFICIAL.copy()
        sim_dh[4, 2] += 1.0
    robot = Robot(args.port, args.baud, sim=args.sim, sim_dh=sim_dh)
    root = tk.Tk()
    App(root, robot, cfg)
    root.mainloop()


if __name__ == "__main__":
    main()
