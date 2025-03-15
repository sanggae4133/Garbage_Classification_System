import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 9600)
time.sleep(2)

try:
    while True:
        ser.write(b'1') 
        time.sleep(2) 
        ser.write(b'0') 
        time.sleep(2) 
except KeyboardInterrupt:
    ser.close() 