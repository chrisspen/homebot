#!/usr/bin/env python

target_speed = -255
current_speed = 0
acceleration = 128 # vel_change/sec
time_set = 0 # ms
current_time = 0 # ms

def update_speed():
    polarity = +1 if target_speed > current_speed else -1
    time_since_set = (current_time - time_set)
    speed_difference = abs(target_speed - current_speed)
    print 'msec to reach goal:', speed_difference/float(acceleration)*1000
    vel_change = time_since_set * (1/1000.) * acceleration * polarity
    new_speed = current_speed + vel_change
    if polarity > 0:
        new_speed = min(new_speed, target_speed)
    else:
        new_speed = max(new_speed, target_speed)
    return new_speed
    
for ms in xrange(0, 3000*2, 100):
    current_time = ms
    _current_speed = update_speed()
    #current_speed = _current_speed
    print 't:%s target speed=%s, current speed=%s' % (current_time, target_speed, _current_speed)