#!/usr/bin/env python
"""
http://letsmakerobots.com/node/31697
"""
import sys
from math import *

degrees_to_radians = pi/180.

def scale(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def get_servo_signal_sin(start_pos, end_pos, speed, current_time):
    #speed = positions/sec
    #current_time = sec since start
    assert 0 <= start_pos <= 180
    assert 0 <= end_pos <= 180
    assert current_time >= 0
    total_time = abs(start_pos - end_pos)/float(speed)
    #print 'total_time:', total_time
    #print 'current_time:', current_time
    current_time_ratio = min(current_time/total_time, 1.0)
    current_time_ratio = scale(current_time_ratio, 0, 1, 0, pi/2)
    #print 'current_time_ratio:', current_time_ratio
    current_pos = sin(current_time_ratio)
    current_pos = scale(current_pos, 0, 1, start_pos, end_pos)
    #print 'current_pos:', current_pos
    current_pos = int(round(current_pos))
    return current_pos

def get_sinus_square(t, steps=180, end=180):
    end = float(end)
    t = float(t)
    v = (steps/pi) * ((pi*t/end) - cos(pi*t/end) * sin(pi*t/end))
#         (steps/pi) * ((pi*t/end) - cos(pi*t/end) * sin(pi*t/end))
#     v = int(round(v))
    return v

def get_servo_signal_sin2(start, end, speed, t):
    assert 0 <= start <= 180
    assert 0 <= end <= 180
    assert t >= 0
    #stop_t = int(abs((end - start)/speed))
    #t = min(stop_t, t)
    dist = end - start
    v = get_sinus_square(t, steps=int(round(dist*float(speed))), end=dist)
    #print 'v:',v
    #v = get_sinus_square(t, steps=dist*speed, end=dist)
#     if(end > start):
#         v += start
#     else:
#         v = start - v
#     v = min(max(v, end), start)
    return v

# assert get_sinus_square(179, steps=180, end=180) == 180

# assert get_servo_signal_sin(start_pos=0, end_pos=180, speed=1, current_time=180) == 180
# assert get_servo_signal_sin(start_pos=0, end_pos=180, speed=2, current_time=90) == 180
# assert get_servo_signal_sin(start_pos=0, end_pos=90, speed=1, current_time=90) == 90

#start_pos = 40; end_pos = 10
# start_pos = 0; end_pos = 180
# speed = 1
# #speed = 2
# for i in xrange(180):
#     print i, get_servo_signal_sin(start_pos, end_pos, speed, i)

def scale(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / float(in_max - in_min) + out_min;

def smootherstep(x, edge0=0., edge1=1.):
    # Scale, bias and saturate x to 0..1 range
    x = min(max((x - edge0)/(edge1 - edge0), 0.0), 1.0);
    # Evaluate polynomial
    #return x*x*(3 - 2*x)
    return x*x*x*(x*(x*6 - 15) + 10)

def servo_signal(start, end, speed, t):
    dist = abs(end - start)
    steps = int(round(dist/float(speed))) + 1
#     for t in xrange(steps):
    #v = (steps/pi) * ((pi*t/end) - cos(pi*t/end) * sin(pi*t/end))
    v = smootherstep(t/float(steps))
    v = int(round(scale(v, 0, 1, start, end)))
    #v = scale(v, 0, 1, start, end)
    #print t, v
    return v

print
start = 90
end = 60
speed = 30 # degree/sec velocity
resolution = 10
for t in xrange(0, (abs(start - end)/speed+1)*1000, 1000/resolution):
    print t, servo_signal(start=start, end=end, speed=speed, t=t/1000.)

# print
# for t in xrange(10+1):
#     print servo_signal(t, start=90, end=60, speed=3)
#
# print
# for t in xrange(30+1):
#     print servo_signal(t, start=90, end=60, speed=1)

sys.exit()
