import pigpio
import time

pi = pigpio.pi()

while True:
    pi.set_servo_pulsewidth(2, 0)
    time.sleep(1)

    pi.set_servo_pulsewidth(2, 600)
    time.sleep(1)

    pi.set_servo_pulsewidth(2, 1500)
    time.sleep(1)

    pi.set_servo_pulsewidth(2, 2400)
    time.sleep(1)