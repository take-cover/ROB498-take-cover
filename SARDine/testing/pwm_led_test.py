#!/usr/bin/env python3
# pwm_led_test.py — visually confirm PWM on pin 32 via LED brightness
import Jetson.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(32, GPIO.OUT)

pwm = GPIO.PWM(32, 1000)  # 1kHz — fast enough for smooth LED dimming
pwm.start(0)

levels = [0, 10, 25, 50, 75, 90, 100]

print("Varying LED brightness via PWM duty cycle...")
print("You should see the LED get progressively brighter then go off\n")

for dc in levels:
    print(f"Duty cycle: {dc}%")
    pwm.ChangeDutyCycle(dc)
    time.sleep(2)

print("\nNow slow fade up and down...")
for dc in list(range(0, 101, 5)) + list(range(100, -1, -5)):
    pwm.ChangeDutyCycle(dc)
    time.sleep(0.05)

pwm.stop()
GPIO.cleanup()
print("Done")