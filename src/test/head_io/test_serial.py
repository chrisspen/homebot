#!/usr/bin/env python
import serial
import time

port = '/dev/ttyACM0'

with serial.Serial(port, 57600, timeout=5) as s:
    i = 0
    while 1:
        i += 1
        
        print 'Writing to Arduino...'
        s.write('ping:' + str(i)+'\n')
        
        print 'Waiting for Arduino...'
        time.sleep(1)
        
        ret = s.readline()
        print 'Received:', ret
