import serial
import time

UP='u'
DOWN='d'
PLASTIC_MOTOR = 0
WASTE_MOTOR = 1
PAPER_MOTOR = 2

py_serial = serial.Serial(
    port = '/dev/ttyUSB0',
    baudrate = 9600,
)

def command(mt_num, mv_type):
    '''
    mt_num: PLASTIC_MOTOR, WASTE_MOTOR, PAPER_MOTOR
    mv_type: UP, DOWN
    '''
    command = str(mv_type) + str(mt_num) + '\n'
    py_serial.write(command.encode())
    time.sleep(0.1)

    if py_serial.readable():
        response = py_serial.readline()
        print(response[:len(response)-1].decode())

while True:
    commend = input()
    py_serial.write((commend + '\n').encode())
    time.sleep(0.1)

    command(PLASTIC_MOTOR, UP)

    if py_serial.readable():
        response = py_serial.readline()
        print(response[:len(response)-1].decode())