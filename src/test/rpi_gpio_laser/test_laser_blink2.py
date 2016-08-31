#!../../../.env/bin/python
# First run doesn't work? Needs to be run twice?
import os
import time
import atexit
import wiringpi2

LASER_EN_PIN = 20 # GPIO 18, (P1 pin 12)

# May give the error:
# bash: echo: write error: Device or resource busy
# if the pin was left exported
cmd = 'cd /sys/class/gpio; echo {pin} > export; echo out > gpio{pin}/direction'.format(pin=LASER_EN_PIN)
#echo 1 > gpio{pin}/value
print cmd
os.system(cmd)

wiringpi2.wiringPiSetupSys()

# Set laser control pin to output.
wiringpi2.pinMode(LASER_EN_PIN, 1)

def cleanup():
    wiringpi2.digitalWrite(LASER_EN_PIN, 0)

atexit.register(cleanup)

delay_sec = 0.1

while 1:
    
    # Turn on laser.
    print 'laser on'
    wiringpi2.digitalWrite(LASER_EN_PIN, 1)
    
    time.sleep(delay_sec)
    
    # Turn off laser.
    print 'laser off'
    wiringpi2.digitalWrite(LASER_EN_PIN, 0)
    
    time.sleep(delay_sec)
