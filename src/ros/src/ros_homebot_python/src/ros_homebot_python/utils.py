import os
import sys
import glob
import commands
import math
from math import pi, sin, cos

import rosnode
from tf import transformations as tf
import numpy as np

from ros_homebot_python import constants as c
from ros_homebot_python.exceptions import *

class Colors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def fail(s):
    return Colors.FAIL + str(s) + Colors.ENDC

def success(s):
    return Colors.OKGREEN + str(s) + Colors.ENDC

#https://gist.github.com/walkermatt/2871026
def debounce(wait):
    """ Decorator that will postpone a functions
        execution until after wait seconds
        have elapsed since the last time it was invoked. """
    import threading
    def decorator(fn):
        def debounced(*args, **kwargs):
            def call_it():
                fn(*args, **kwargs)
            try:
                debounced.t.cancel()
            except(AttributeError):
                pass
            debounced.t = threading.Timer(wait, call_it)
            debounced.t.start()
        return debounced
    return decorator

def find_serial_device(name, base_path='/dev/ttyA*', verbose=False):
    """
    Given the name of the serial device to find, returns the corresponding /dev/ttyACM* path.
    """
    assert name in c.DEVICE_SIGNATURES, 'Unknown name: %s' % name
    if verbose:
        print '-'*80
        print 'Attempting to find %s...' % name
    device_paths = glob.glob(base_path)
    for device_path in device_paths:
        if verbose:
            print 'Checking %s...' % device_path
        cmd = 'udevadm info --query=all --name={path}'.format(**{'path': device_path})
        if verbose:
            print 'cmd:', cmd
        ret = commands.getoutput(cmd)
        ret = ret.lower()
        if verbose:
            print 'ret:', ret
        device_sigs = c.DEVICE_SIGNATURES[name]
        for device_sig in device_sigs:
            if device_sig.lower() in ret:
                if verbose:
                    print 'Identified %s as %s!' % (device_path, name)
                return device_path
        if verbose:
            print 'Unable to identify %s!' % device_path
                    
    raise DeviceNotFound, (
        'Device %s not found. '
        'Ensure device is connected and that base_path %s is correct?'
    ) % (name, base_path)

def to_10(data):
    data = str(data).lower().strip()
    if data in ('1', 'on', 'true'):
        data = '1'
    elif data in ('0', 'off', 'false'):
        data = '0'
    else:
        raise ValueError
    return data

def R(theta, u):
    return [[cos(theta) + u[0]**2 * (1-cos(theta)), 
             u[0] * u[1] * (1-cos(theta)) - u[2] * sin(theta), 
             u[0] * u[2] * (1 - cos(theta)) + u[1] * sin(theta)],
            [u[0] * u[1] * (1-cos(theta)) - u[2] * sin(theta),
             cos(theta) + u[1]**2 * (1-cos(theta)),
             u[1] * u[2] * (1 - cos(theta)) + u[0] * sin(theta)],
            [u[0] * u[2] * (1-cos(theta)) - u[1] * sin(theta),
             u[1] * u[2] * (1-cos(theta)) - u[0] * sin(theta),
             cos(theta) + u[2]**2 * (1-cos(theta))]]

#http://stackoverflow.com/a/17763844/247542
def rotate(pointToRotate, point1, point2, theta):

    u = []
    squaredSum = 0
    for i,f in zip(point1, point2):
        u.append(f-i)
        squaredSum += (f-i) **2

    u = [i/squaredSum for i in u]

    r = R(theta, u)
    rotated = []

    for i in range(3):
        rotated.append(round(sum([r[j][i] * pointToRotate[j] for j in range(3)])))

    return rotated

def head_angles_to_point(pan, tilt, distance):
    """
    pan and tilt are in degrees
    """
    
    # Verify hard tilt endstops. 
    if c.TILT_MIN > tilt or c.TILT_MAX < tilt:
         
        return None, None, None
         
    # Tilt angle is centered a 90, so scale to make 90=0.
    tilt -= 90
    print 'tilt0:', tilt
    
    # Pan has no hard endstops, but ensure stays within relative limit.
    pan = pan % c.PAN_MAX
    
    origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
    
    # The initial point is the distance projected from the face of the camera on the head.
    # The positive x-axis extends from the face.
    # The positive y-axis extends from the right-hand side facing outwards from the face.
    point = np.array([c.TORSO_DIAMETER_MM + distance, 0, c.HEIGHT_CENTER_HEIGHT_MM, 0])
    print 'point0:', point
    
    # Apply the tilt angle by rotating about the y-axis of the head.
    if tilt:
        #point = rotate(point, (0, 0, c.HEIGHT_CENTER_HEIGHT_MM), (0, 10, c.HEIGHT_CENTER_HEIGHT_MM), tilt*pi/180.)
        point = np.add((0, 0, -c.HEIGHT_CENTER_HEIGHT_MM, 0), point)
        print 'point1:', point
        point = tf.rotation_matrix(-tilt*pi/180., yaxis).dot(point)
        print 'point2:', point
        point = np.add((0, 0, +c.HEIGHT_CENTER_HEIGHT_MM, 0), point)
        print 'point3:', point
    
    # Apply the pan angle by rotating about the z-axis.
    if pan:
        #point = rotate(point, (0, 0, 0), (0, 0, 1), pan*pi/180.)
        point = tf.rotation_matrix(pan*pi/180., zaxis).dot(point)
        pass
    
    return tuple(point[:3])

def head_point_to_angles(point):
    pass

def relative_to_absolute_speed(relative):
    assert -c.MOTOR_MAX_SPEED <= relative <= c.MOTOR_MAX_SPEED
    absolute_speed = c.MOTOR_MAX_SPEED_REAL/c.MOTOR_MAX_SPEED*relative
    return absolute_speed

def linear_travel_time(speed, distance, start_speed=None, end_speed=None, accel=None):
    """
    Calculates the total time needed to travel a distance at a given speed.
    """
    accel = accel or c.MOTOR_DEFAULT_ACCEL_REAL
    
    t1 = 0 * c.SEC
    d1 = 0 * c.MM
    if start_speed is not None:
        # How long will it take to transition from start_speed to speed?
        # v = v0  + a*t => t = (v - v0)/a
        t1 = abs(speed - start_speed)/accel
        # How much distance have we travelled during acceleration?
        # s = vi*t + 0.5*a*t^2
        d1 = start_speed*t1 + 0.5*accel*t1**2
    
    t2 = 0 * c.SEC
    d2 = 0 * c.MM
    if end_speed is not None:
        # How long will it take to transition from start_speed to speed?
        # v = v0  + a*t => t = (v - v0)/a
        t2 = abs(speed - end_speed)/accel
        # How much distance have we travelled during acceleration?
        # s = vi*t + 0.5*a*t^2
        d2 = end_speed*t2 + 0.5*accel*t2**2
        
    speed = speed.to(c.MM/c.SEC)
    distance = distance.to(c.MM)
    t = abs((distance - d1 - d2) / speed) + t1 + t2
    return t

def rotational_travel_time(speed, degrees):
    speed = speed.to(c.MM/c.SEC)
    degrees = degrees.to(c.DEG)
    circumference = math.pi*c.TORSO_TREAD_WIDTH
    distance = circumference * degrees/(360. * c.DEG)
    return abs(distance / speed)

def assert_node_alive(name):
    print('Pinging %s...' % name)
    assert rosnode.rosnode_ping(name, max_count=1, verbose=True), 'Node %s node not detected.' % name

def get_angle_of_pixel(pixel, resolution, angle_of_view):
    """
    Calculates a pixels's angle from the origin.
    
    e.g. A pixel at position 0 would be at angle 0, and a pixel at position=resolution
    would be at angle=angle_of_view.
    """
    return angle_of_view/float(resolution)*pixel
