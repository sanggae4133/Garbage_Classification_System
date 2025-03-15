import serial
import time

py_serial = serial.Serial(
    port = '/dev/ttyUSB0',
    baudrate = 9600,
)

while True:
    commend = input('the things you want arduino to do for you:')
    py_serial.write(commend.encode())
    time.sleep(0.1)

    if py_serial.readable():
        response = py_serial.readline()
        print(response[:len(response)-1].decode())