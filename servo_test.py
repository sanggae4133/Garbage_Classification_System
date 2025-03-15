import serial
import time

py_serial = serial.Serial(
    port = '/dev/ttyUSB0',
    baudrate = 9600,
)

while True:
    commend = input()
    py_serial.write((commend + '\n').encode())
    time.sleep(0.1)

    if py_serial.readable():
        response = py_serial.readline()
        print(response[:len(response)-1].decode())