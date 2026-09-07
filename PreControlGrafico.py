import tkinter as tk
from pymycobot.mycobot import MyCobot

# 1. Configuración de conexión serial
# En la Raspberry Pi, el puerto suele ser ttyAMA0, pero puede variar a ttyUSB0
PORT = '/dev/ttyAMA0'
BAUD = 115200

# Conectar al brazo (Si estás probando el código sin el brazo, comenta el bloque try-except)
try:
    mc = MyCobot(PORT, BAUD)
    mc.power_on()
except Exception as e:
    print(f"Error de conexión. Verifica el puerto: {e}")
    mc = None

def update_angles(val):
    """Envía la matriz de ángulos al brazo cuando se mueve un deslizador."""
    if mc is None:
        return
        
    angles = [
        j1_slider.get(),
        j2_slider.get(),
        j3_slider.get(),
        j4_slider.get(),
        j5_slider.get(),
        j6_slider.get()
    ]
    # Velocidad de movimiento (0-100). Mantenerla baja por seguridad durante pruebas.
    speed = 40
    mc.send_angles(angles, speed)

def release_motors():
    """Libera los servos para que el brazo quede flácido (útil por seguridad)."""
    if mc:
        mc.release_all_servos()

# 2. Construcción de la Interfaz Gráfica
root = tk.Tk()
root.title("MyCobot 320 Pi - Control J1-J6")
root.geometry("400x500")

tk.Label(root, text="Control Angular (Grados)", font=("Arial", 14, "bold")).pack(pady=15)

# Limites aproximados de seguridad en grados para evitar colisiones del propio brazo
# (J1: Base, J2: Hombro, J3: Codo, J4: Muñeca pitch, J5: Muñeca yaw, J6: Muñeca roll)
limits = [
    (-160, 160),
    (-160, 160),
    (-160, 160),
    (-160, 160),
    (-160, 160),
    (-175, 175)
]

sliders_list = []
for i in range(6):
    frame = tk.Frame(root)
    frame.pack(fill='x', padx=20, pady=8)
    
    tk.Label(frame, text=f"J{i+1}:", font=("Arial", 12)).pack(side='left')
    
    # Se pasa 'update_angles' al comando para que se ejecute al soltar/mover el control
    slider = tk.Scale(frame, from_=limits[i][0], to=limits[i][1],
                      orient='horizontal', length=250, command=update_angles)
    slider.set(0) # Todos los motores inician en la posición cero
    slider.pack(side='right')
    sliders_list.append(slider)

# Asignación individual para la función update_angles
j1_slider, j2_slider, j3_slider, j4_slider, j5_slider, j6_slider = sliders_list

# Botón de emergencia/relajación
tk.Button(root, text="Liberar Motores (Relajado)", bg="red", fg="white",
          font=("Arial", 12), command=release_motors).pack(pady=25)

root.mainloop()