import RPi.GPIO as GPIO
import time

# Motor GPIO pins
LEFT_MOTOR = 17
RIGHT_MOTOR = 18

# Ultrasonic sensor GPIO pins
TRIG = 23
ECHO = 24

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(LEFT_MOTOR, GPIO.OUT)
GPIO.setup(RIGHT_MOTOR, GPIO.OUT)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def get_distance():
    GPIO.output(TRIG, False)
    time.sleep(0.1)

    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()

    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Convert to cm
    return distance

def drive_forward():
    GPIO.output(LEFT_MOTOR, True)
    GPIO.output(RIGHT_MOTOR, True)

def stop():
    GPIO.output(LEFT_MOTOR, False)
    GPIO.output(RIGHT_MOTOR, False)

try:
    while True:
        dist = get_distance()
        print(f"Distance: {dist:.2f} cm")
        if dist < 30:  # Stop if object is closer than 30 cm
            stop()
            print("Object detected. Stopping.")
            break
        else:
            drive_forward()
        time.sleep(0.1)

except KeyboardInterrupt:
    GPIO.cleanup()
