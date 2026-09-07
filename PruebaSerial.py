import time
from pymycobot.mycobot320 import MyCobot320

puerto = '/dev/ttyAMA0'
baudrate = 115200

print(f"puerto: {puerto}...")

try:
    mc = MyCobot320(puerto, baudrate)
    
    time.sleep(1)
    
    print("Estado de los servos:    ")
    angulos = mc.get_angles()
    
    if angulos:
        print(f"Valores son: {angulos}")
        
    else:
        print("No")
        
except PermissionError:
    print("error en permisos")
    
except Exception as e:
    print(f"error en comunicacion  {e}")