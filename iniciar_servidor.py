from pymycobot.mycobotserver import MyCobotServer

# Parámetros: Puerto serial interno, Baudios, IP local, Puerto de red
puerto_serial = '/dev/ttyAMA0'
baudios = 115200
ip = "0.0.0.0"  # Permite conexiones de cualquier dispositivo en tu red WiFi/Ethernet
puerto_red = 9000

print(f"Iniciando servidor en el puerto {puerto_red}... (Presiona Ctrl+C para detener)")

# Se inicializa y arranca el puente de comunicación
server = MyCobotServer(puerto_serial, baudios, ip, puerto_red)
server.start()
