#!/usr/bin/env python3
# servo_sweep.py — sweeps full duty cycle range to find your servo's working values
import Jetson.GPIO as GPIO
import time

SERVO_PIN = 32
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(SERVO_PIN, GPIO.OUT)

pwm = GPIO.PWM(SERVO_PIN, 50)
pwm.start(0)
time.sleep(0.5)

print("Sweeping from 2% to 15% duty cycle...")
print("Watch your servo and note where it starts and stops moving\n")

for dc in [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]:
    print(f"Duty cycle: {dc}%")
    pwm.ChangeDutyCycle(dc)
    time.sleep(1.5)

pwm.stop()
GPIO.cleanup()
print("\nDone. Note the lowest and highest DC values where the servo moved.")