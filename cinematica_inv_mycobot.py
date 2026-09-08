import tkinter as tk
from tkinter import ttk
from pymycobot import MyCobot320

PORT = '/dev/ttyAMA0'
BAUD = 115200

try:
    mc = MyCobot320(PORT, BAUD)
    mc.power_on()
    print("Robot conectado correctamente")
except Exception as e:
    print(f"Error de conexión: {e}")
    mc = None

routine_points = []


# ============================================================
# FUNCIONES DE CONTROL ARTICULAR
# ============================================================

def update_angles(val=None):
    if mc:
        try:
            angles = [s.get() for s in angle_sliders]

            mc.send_angles(
                angles,
                speed_var.get()
            )

        except Exception as e:
            print("Error enviando ángulos:", e)


# ============================================================
# FUNCIONES DE CONTROL CARTESIANO
# ============================================================

def update_coords(val=None):
    if mc:
        try:

            coords = [
                s.get()
                for s in coord_sliders
            ]

            print(
                "Pose enviada:",
                [round(v, 2) for v in coords]
            )

            # send_coords:
            #
            # [X, Y, Z, RX, RY, RZ]
            #
            # mode = 1 -> movimiento lineal cartesiano

            mc.send_coords(
                coords,
                speed_var.get(),
                1
            )

        except Exception as e:
            print("Error enviando coordenadas:", e)


def read_coords():
    """
    Lee la pose cartesiana REAL del robot
    y coloca los sliders en esos valores.
    """

    if mc:
        try:

            coords = mc.get_coords()

            if coords and len(coords) == 6:

                print(
                    "Pose actual:",
                    coords
                )

                for i in range(6):
                    coord_sliders[i].set(coords[i])

                update_pose_label(coords)

        except Exception as e:
            print("Error leyendo coordenadas:", e)


def update_pose_label(coords=None):

    if coords is None:
        coords = [
            s.get()
            for s in coord_sliders
        ]

    pose_text.set(
        f"X = {coords[0]:7.1f} mm      "
        f"Y = {coords[1]:7.1f} mm      "
        f"Z = {coords[2]:7.1f} mm\n"
        f"RX = {coords[3]:7.1f}°        "
        f"RY = {coords[4]:7.1f}°        "
        f"RZ = {coords[5]:7.1f}°"
    )


# ============================================================
# HOME
# ============================================================

def go_home():

    if mc:
        try:

            mc.send_angles(
                [0, 0, 0, 0, 0, 0],
                speed_var.get()
            )

            for slider in angle_sliders:
                slider.set(0)

        except Exception as e:
            print("Error enviando HOME:", e)


# ============================================================
# LIBERAR MOTORES
# ============================================================

def release_motors():

    if mc:
        try:

            mc.release_all_servos()

            print("Servomotores liberados")

        except Exception as e:
            print("Error liberando motores:", e)


# ============================================================
# ACTIVAR MOTORES
# ============================================================

def enable_motors():

    if mc:
        try:

            mc.focus_all_servos()

            print("Servomotores activados")

        except Exception as e:
            print("Error activando motores:", e)


# ============================================================
# GUARDAR PUNTO
# ============================================================

def save_point():

    if mc:
        try:

            current_angles = mc.get_angles()

            if current_angles:

                routine_points.append(
                    current_angles
                )

                routine_listbox.insert(
                    tk.END,
                    f"P {len(routine_points)}: "
                    f"{[round(v, 1) for v in current_angles]}"
                )

        except Exception as e:
            print("Error guardando punto:", e)


# ============================================================
# EJECUTAR RUTINA
# ============================================================

def play_routine(index=0):

    if mc and index < len(routine_points):

        try:

            mc.send_angles(
                routine_points[index],
                speed_var.get()
            )

            root.after(
                2500,
                play_routine,
                index + 1
            )

        except Exception as e:
            print("Error ejecutando rutina:", e)


def clear_routine():

    routine_points.clear()

    routine_listbox.delete(
        0,
        tk.END
    )


# ============================================================
# INTERFAZ
# ============================================================

root = tk.Tk()

root.title(
    "MyCobot 320 Pi - Control de Posición y Orientación"
)

root.geometry(
    "600x720"
)


# ============================================================
# PANEL GLOBAL
# ============================================================

global_frame = tk.Frame(root)

global_frame.pack(
    fill='x',
    pady=10,
    padx=20
)


tk.Label(
    global_frame,
    text="Velocidad:",
    font=("Arial", 10, "bold")
).pack(side='left')


speed_var = tk.IntVar(
    value=30
)


speed_slider = tk.Scale(
    global_frame,
    from_=1,
    to=100,
    orient='horizontal',
    variable=speed_var,
    length=180
)

speed_slider.pack(
    side='left',
    padx=10
)


tk.Button(
    global_frame,
    text="HOME",
    bg="lightblue",
    command=go_home
).pack(
    side='right'
)


# ============================================================
# PESTAÑAS
# ============================================================

notebook = ttk.Notebook(root)

notebook.pack(
    pady=10,
    expand=True,
    fill='both'
)


# ============================================================
# PESTAÑA 1
# ARTICULACIONES
# ============================================================

tab_angles = ttk.Frame(notebook)

notebook.add(
    tab_angles,
    text="Articulaciones"
)


angle_limits = [

    (-168, 168),   # J1
    (-135, 135),   # J2
    (-145, 145),   # J3
    (-148, 148),   # J4
    (-168, 168),   # J5
    (-180, 180)    # J6

]


angle_sliders = []


for i, limits in enumerate(angle_limits):

    frame = tk.Frame(tab_angles)

    frame.pack(
        fill='x',
        padx=10,
        pady=5
    )

    tk.Label(
        frame,
        text=f"J{i+1}:",
        width=5,
        font=("Arial", 10, "bold")
    ).pack(
        side='left'
    )


    slider = tk.Scale(
        frame,
        from_=limits[0],
        to=limits[1],
        resolution=1,
        orient='horizontal',
        length=420,
        command=update_angles
    )

    slider.set(0)

    slider.pack(
        side='right'
    )

    angle_sliders.append(
        slider
    )


# ============================================================
# PESTAÑA 2
# POSICIÓN Y ORIENTACIÓN
# ============================================================

tab_coords = ttk.Frame(notebook)

notebook.add(
    tab_coords,
    text="Posición / Orientación"
)


# Límites de trabajo
# X,Y,Z en mm
# RX,RY,RZ en grados

coord_limits = [

    (-350, 350),     # X
    (-350, 350),     # Y
    (-41, 523.9),    # Z

    (-180, 180),     # RX
    (-180, 180),     # RY
    (-180, 180)      # RZ

]


coord_labels = [

    "X",
    "Y",
    "Z",

    "RX",
    "RY",
    "RZ"

]


coord_units = [

    "mm",
    "mm",
    "mm",

    "°",
    "°",
    "°"

]


coord_sliders = []


# ============================================================
# TÍTULO POSICIÓN
# ============================================================

tk.Label(
    tab_coords,
    text="POSICIÓN DEL EFECTOR FINAL",
    font=("Arial", 11, "bold")
).pack(
    pady=(10, 2)
)


for i in range(3):

    frame = tk.Frame(tab_coords)

    frame.pack(
        fill='x',
        padx=10,
        pady=3
    )

    tk.Label(
        frame,
        text=coord_labels[i],
        width=5,
        font=("Arial", 10, "bold")
    ).pack(
        side='left'
    )


    slider = tk.Scale(
        frame,
        from_=coord_limits[i][0],
        to=coord_limits[i][1],

        resolution=1,

        orient='horizontal',

        length=400,

        command=lambda val: (
            update_pose_label(),
            update_coords()
        )
    )


    if coord_labels[i] == "Z":
        slider.set(200)
    else:
        slider.set(0)


    slider.pack(
        side='left'
    )


    tk.Label(
        frame,
        text=coord_units[i],
        width=4
    ).pack(
        side='left'
    )


    coord_sliders.append(
        slider
    )


# ============================================================
# TÍTULO ORIENTACIÓN
# ============================================================

ttk.Separator(
    tab_coords,
    orient='horizontal'
).pack(
    fill='x',
    pady=8,
    padx=20
)


tk.Label(
    tab_coords,
    text="ORIENTACIÓN DEL EFECTOR FINAL",
    font=("Arial", 11, "bold")
).pack(
    pady=(2, 2)
)


for i in range(3, 6):

    frame = tk.Frame(tab_coords)

    frame.pack(
        fill='x',
        padx=10,
        pady=3
    )

    tk.Label(
        frame,
        text=coord_labels[i],
        width=5,
        font=("Arial", 10, "bold")
    ).pack(
        side='left'
    )


    slider = tk.Scale(
        frame,
        from_=coord_limits[i][0],
        to=coord_limits[i][1],

        resolution=1,

        orient='horizontal',

        length=400,

        command=lambda val: (
            update_pose_label(),
            update_coords()
        )
    )


    slider.set(0)


    slider.pack(
        side='left'
    )


    tk.Label(
        frame,
        text=coord_units[i],
        width=4
    ).pack(
        side='left'
    )


    coord_sliders.append(
        slider
    )


# ============================================================
# MOSTRAR POSE
# ============================================================

pose_text = tk.StringVar()


pose_label = tk.Label(
    tab_coords,
    textvariable=pose_text,
    font=("Consolas", 11),
    bg="white",
    relief="sunken",
    padx=10,
    pady=8
)


pose_label.pack(
    fill="x",
    padx=20,
    pady=10
)


update_pose_label()


# ============================================================
# BOTÓN LEER POSE ACTUAL
# ============================================================

tk.Button(
    tab_coords,
    text="Leer posición y orientación actual",
    bg="lightgreen",
    font=("Arial", 10, "bold"),
    command=read_coords
).pack(
    pady=8
)


# ============================================================
# PESTAÑA 3
# RUTINAS
# ============================================================

tab_routine = ttk.Frame(notebook)

notebook.add(
    tab_routine,
    text="Rutinas"
)


tk.Label(
    tab_routine,
    text=(
        "1. Libera los motores.\n"
        "2. Mueve el brazo manualmente.\n"
        "3. Guarda los puntos.\n"
        "4. Ejecuta la trayectoria."
    ),
    justify="left"
).pack(
    pady=10
)


btn_frame = tk.Frame(tab_routine)

btn_frame.pack(
    fill='x',
    padx=20
)


tk.Button(
    btn_frame,
    text="Guardar Punto",
    bg="lightgreen",
    command=save_point
).pack(
    side='left',
    expand=True,
    fill='x',
    padx=5
)


tk.Button(
    btn_frame,
    text="Ejecutar Rutina",
    bg="gold",
    command=play_routine
).pack(
    side='left',
    expand=True,
    fill='x',
    padx=5
)


tk.Button(
    btn_frame,
    text="Limpiar",
    command=clear_routine
).pack(
    side='left',
    expand=True,
    fill='x',
    padx=5
)


routine_listbox = tk.Listbox(
    tab_routine,
    height=15
)

routine_listbox.pack(
    fill='both',
    expand=True,
    padx=20,
    pady=10
)


# ============================================================
# CONTROL MOTORES
# ============================================================

motor_frame = tk.Frame(root)

motor_frame.pack(
    fill='x',
    padx=20,
    pady=10
)


tk.Button(
    motor_frame,
    text="Activar Motores",
    bg="lightgreen",
    font=("Arial", 11, "bold"),
    command=enable_motors
).pack(
    side='left',
    expand=True,
    fill='x',
    padx=5
)


tk.Button(
    motor_frame,
    text="Liberar Motores",
    bg="red",
    fg="white",
    font=("Arial", 11, "bold"),
    command=release_motors
).pack(
    side='left',
    expand=True,
    fill='x',
    padx=5
)


# ============================================================
# INICIAR GUI
# ============================================================

root.mainloop()