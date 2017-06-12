#!../../../.env/bin/python
#DO NOT USE, RPi.GPIO directly accesses /dev/mem, bad practice
import time
# import atexit

import RPi.GPIO as GPIO

LASER_EN_PIN = 18 # GPIO 18, (P1 pin 12)

print 'A'
GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme
print 'B'
GPIO.setup(LASER_EN_PIN, GPIO.OUT) # Laser pin output
print 'C'
GPIO.output(LASER_EN_PIN, GPIO.LOW) # Lasert off

# def cleanup():
#     GPIO.cleanup()
#
# atexit.register(cleanup)

try:
    while 1:

        # Turn on laser.
        print 'D'
        GPIO.output(LASER_EN_PIN, GPIO.HIGH)

        time.sleep(1)

        # Turn off laser.
        print 'E'
        GPIO.output(LASER_EN_PIN, GPIO.LOW)

        time.sleep(1)
except KeyboardInterrupt:
    GPIO.cleanup()
