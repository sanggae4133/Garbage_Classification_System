import sys
import time
import RPi.GPIO as GPIO

# Category to type mapping
CATEGORY_MAP = {
    "Cup": "plastic",
    "Carton": "paper",
    "Styrofoam piece": "plastic",
    "Plastic gloves": "plastic",
    "Broken glass": "glass",
    "Battery": "garbage",
    "Straw": "plastic",
    "Blister pack": "garbage",
    "Plastic bag & wrapper": "plastic",
    "Paper": "paper",
    "Can": "can",
    "Lid": "plastic",
    "Plastic container": "plastic",
    "Plastic utensils": "plastic",
    "Bottle cap": "plastic",
    "Paper bag": "paper",
    "Rope & strings": "plastic",
    "Food waste": "garbage",
    "Scrap metal": "can",
    "Bottle": "plastic",
    "Glass jar": "glass",
    "Squeezable tube": "garbage",
    "Aluminium foil": "can",
    "Clear plastic bottle": "plastic",
    "Other plastic bottle": "plastic",
    "Glass bottle": "glass"
}

# Type to GPIO pin mapping
SERVO_PIN_MAP = {
    "garbage": 2,
    "plastic": 4,
    "glass": 6,
    "can": 27,
    "paper": 25
}

SERVO_MAX_DUTY = 12
SERVO_MIN_DUTY = 3

# Set GPIO mode
GPIO.setmode(GPIO.BCM)
servo_motors = {}  # Dictionary to store servo motor objects

# Initialize servo motors
def initialize_servos():
    for servo_type, pin in SERVO_PIN_MAP.items():
        if pin not in servo_motors:  # Avoid duplicate initialization
            GPIO.setup(pin, GPIO.OUT)
            motor = GPIO.PWM(pin, 50)  # 50Hz PWM frequency
            motor.start(0)  # Initial position 0 degrees
            servo_motors[pin] = motor  # Store PWM object by pin

# Set servo motor position
def setServoPos(servo, degree):
    if degree > 180:
        degree = 180

    duty = SERVO_MIN_DUTY + (degree * (SERVO_MAX_DUTY - SERVO_MIN_DUTY) / 180.0)
    print(f"Degree: {degree} to {duty}(Duty)")
    servo.ChangeDutyCycle(duty)

# Operate specific servo motor based on category
def operate_servo(category):
    servo_type = CATEGORY_MAP.get(category)
    pin = SERVO_PIN_MAP.get(servo_type)

    if pin and pin in servo_motors:
        print(f"Operating servo motor for {category} ({servo_type}) on pin {pin}")
        setServoPos(servo_motors[pin], 90)  # Rotate to 90 degrees
        time.sleep(1)
        setServoPos(servo_motors[pin], 0)   # Return to original position
    else:
        print(f"No servo motor assigned for {category}.")

# Clean up GPIO resources
def cleanup_servos():
    print("Cleaning up servo motors...")
    for motor in servo_motors.values():
        motor.stop()
    GPIO.cleanup()

# Main function
if __name__ == '__main__':
    try:
        initialize_servos()  # Initialize servo motors

        print("Pin testing")
        print("Testing plastic on pin 4")
        operate_servo("Cup")
        time.sleep(5)

        print("Testing paper on pin 22")
        operate_servo("Carton")
        time.sleep(5)

        print("Testing glass on pin 6")
        operate_servo("Broken glass")
        time.sleep(5)

        print("Testing garbage on pin 2")
        operate_servo("Blister pack")
        time.sleep(5)

        print("Testing paper on pin 22")
        operate_servo("Paper")
        time.sleep(5)

        print("Testing can on pin 27")
        operate_servo("Can")
        time.sleep(5)

        print("=== Pin test end ===")

        '''
        # Test all servo motors by category
        for category in CATEGORY_MAP:
            print(f"Testing: {category}")
            operate_servo(category)
            time.sleep(0.5)  # Wait before the next motor operation
        '''

    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        cleanup_servos()  # Clean up resources on program exit