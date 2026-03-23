#!/usr/bin/env python3
# servo_test.py — no ROS needed, run directly with: python3 servo_test.py
import Jetson.GPIO as GPIO
import time

SERVO_PIN = 32
GPIO.setmode(GPIO.BOARD)
GPIO.setup(SERVO_PIN, GPIO.OUT)

pwm = GPIO.PWM(SERVO_PIN, 50)
pwm.start(7.5)  # home position
print("Servo at HOME (7.5%) — you should hear/feel it settle")
time.sleep(2)

print("Servo moving to DROP (10.0%)")
pwm.ChangeDutyCycle(10.0)
time.sleep(2)

print("Servo returning to HOME (7.5%)")
pwm.ChangeDutyCycle(7.5)
time.sleep(2)

pwm.stop()
GPIO.cleanup()
print("Done")