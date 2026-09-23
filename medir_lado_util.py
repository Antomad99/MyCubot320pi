import time
import math
from pymycobot import MyCobot320

PORT = '/dev/ttyAMA0'
BAUD = 115200
SPEED = 25  # despacio, para revisar con cuidado

# Los 4 puntos que guardaste (J1..J6, en grados), tal como aparecen en tu captura.
saved_angle_points = [
    [9.66, -49.48, -63.89, 5.53, 99.22, 0.26],
    [12.48, -52.82, -91.4, 69.87, 95.8, -0.08],
    [5.27, -65.47, -81.91, 81.82, 109.51, 1.14],
    [13.18, -59.58, -93.77, 61.96, 80.77, 1.14],
]

def distancia(c1, c2):
    return math.sqrt(sum((c1[i] - c2[i])**2 for i in range(3)))

def main():
    mc = MyCobot320(PORT, BAUD)
    mc.power_on()

    coords_reales = []
    for i, angles in enumerate(saved_angle_points):
        print(f"Moviendo a P{i+1}: {angles}")
        mc.send_angles(angles, SPEED)
        time.sleep(3)  # ajusta si tu robot tarda más en llegar
        c = mc.get_coords()
        if not c:
            print(f"  No se pudo leer get_coords() en P{i+1}, revisa la conexión.")
            continue
        coords_reales.append(c)
        print(f"  Coordenadas reales: X={c[0]:.1f} Y={c[1]:.1f} Z={c[2]:.1f} "
              f"RX={c[3]:.1f} RY={c[4]:.1f} RZ={c[5]:.1f}")

    if len(coords_reales) < 2:
        print("No hay suficientes puntos válidos para medir distancias.")
        return

    print("\n--- Distancias entre puntos consecutivos (mm) ---")
    for i in range(len(coords_reales) - 1):
        d = distancia(coords_reales[i], coords_reales[i+1])
        print(f"P{i+1} -> P{i+2}: {d:.1f} mm")

    if len(coords_reales) == 4:
        diag1 = distancia(coords_reales[0], coords_reales[2])
        diag2 = distancia(coords_reales[1], coords_reales[3])
        print(f"\nDiagonales: P1-P3 = {diag1:.1f} mm | P2-P4 = {diag2:.1f} mm")
        print("(Si el plano es aprox. rectangular, las diagonales deberían ser similares entre sí.)")

if __name__ == "__main__":
    main()
