import time
from pymycobot.mycobot320 import MyCobot320

puerto = '/dev/ttyAMA0'
baudrate = 115200

print(f"puerto: {puerto}...")

try:
    mc = MyCobot320(puerto, baudrate)
    time.sleep(1)

    mc.power_on()
    time.sleep(2)

    mc.is_all_servo_enable()
    time.sleep(1)

    print("Iniciando mov ")
    mc.send_angles([30,30,30,30,30,30],50)

    print("mov m1 ")
    #mc.send_angle(1,45,50)
    time.sleep(1)

    print("Iniciando mov ")
    mc.send_angles([0,0,0,0,0,0],50)
    time.sleep(2)
    
    time.sleep(5)
    print("En posicion ")
    print(f"Angulos actuales: {mc.get_angles()}")
except Exception as e:
    print(f"Error: {e}")