#!/usr/bin/env python
# -----------
# User Instructions
#
# Implement a P controller by running 100 iterations
# of robot motion. The steering angle should be set
# by the parameter tau so that:
#
# steering = -tau_p * CTE - tau_d * diff_CTE - tau_i * int_CTE
#
# where the integrated crosstrack error (int_CTE) is
# the sum of all the previous crosstrack errors.
# This term works to cancel out steering drift.
#
# Your code should print a list that looks just like
# the list shown in the video.
#
# Only modify code at the bottom!
# ------------

from __future__ import print_function

import unittest
from math import *
import random

import turtle

import PID

# ------------------------------------------------
#
# this is the robot class
#

def rotate(x, y, theta):
    xp = x*cos(theta) - y*sin(theta)
    yp = x*sin(theta) + y*cos(theta)
    return xp, yp

def dist(a, b):
    return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

class robot:

    # --------
    # init:
    #    creates robot and initializes location/orientation to 0, 0, 0
    #

    def __init__(self, length = 20.0):
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
#         self.length = length
        self.steering_noise = 0.0
        self.distance_noise = 0.0
        self.steering_drift = 0.0

    def set(self, new_x, new_y, new_orientation):
        """
        sets a robot coordinate
        """
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation) % (2.0 * pi)

    def set_noise(self, new_s_noise, new_d_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.steering_noise = float(new_s_noise)
        self.distance_noise = float(new_d_noise)

    def set_steering_drift(self, drift):
        self.steering_drift = drift

    def move(self, steering, distance, treadmill_speed=None, tolerance = 0.001, max_steering_angle = pi / 4.0):
        """
           steering = deflection angle in parallel wheel speeds, in radians
           distance = total distance driven, most be non-negative
        """

        if steering > max_steering_angle:
            steering = max_steering_angle
        if steering < -max_steering_angle:
            steering = -max_steering_angle
        if distance < 0.0:
            distance = 0.0

        # make a new copy
        res = robot()
#         res.length         = self.length
        res.steering_noise = self.steering_noise
        res.distance_noise = self.distance_noise
        res.steering_drift = self.steering_drift

        # apply noise
        steering2 = random.gauss(steering, self.steering_noise)
        distance2 = random.gauss(distance, self.distance_noise)

        # apply steering drift
        steering2 += self.steering_drift

        # Execute motion
        #turn = tan(steering2) * distance2 / res.length
        turn = steering2

        # The old turn radians.
        theta0 = self.orientation

        # Calculate initial vector.
        x, y = (distance2, 0)
        x, y = rotate(x, y, theta0)

        # The new turn radians.
        theta1 = turn

        # Calculate new final turn
        theta2 = theta0 + theta1

        # Calculate new vector.
        x, y = rotate(x, y, theta1)
        res.x = self.x + x
        res.y = self.y + y
        res.orientation = theta2 % (2.0 * pi)

#         if treadmill_speed:
#             res.x += treadmill_speed[0]
#             res.y += treadmill_speed[1]

        return res, (distance2, turn)

    def __repr__(self):
        return '[x=%.5f y=%.5f orient=%.5f]'  % (self.x, self.y, self.orientation)

############## ADD / MODIFY CODE BELOW ####################

# ------------------------------------------------------------------------
#
# run - does a single control run.

class Tests(unittest.TestCase):

    def test_rotate(self):

        x, y = rotate(0, 1, -pi/2.)
        self.assertAlmostEqual(x, 1)
        self.assertAlmostEqual(y, 0)

    def test_robot_turn(self):
        turn_degrees = 2
        param1, param2, param3 = 0., 3.0, 0.004
        myrobot = robot()
#         myrobot.set_noise(0.01, 0.01)
#         myrobot.set_steering_drift(0.001)
        myrobot.set(new_x=0.0, new_y=0.0, new_orientation=0.0)
        speed = 1.0 # motion distance is equal to speed (we assume time = 1)
        N = 300

        tau_p = param1
        tau_d = param2
        tau_i = param3
        #Y_last = None
        #Y_last = myrobot.y - 0
        cte_sum = 0
        alpha = 0
        for step in xrange(N):
            #Y = myrobot.y - 0
            #cte_sum += Y#should be here?
            #alpha = -tau_p * Y - tau_d * (Y - Y_last) - tau_i * cte_sum
            #+alpha=left, -alpha=right

            alpha = turn_degrees * pi / 180.
            myrobot, (distance2, od) = myrobot.move(steering=alpha, distance=speed)
            print(myrobot, alpha)
            #Y_last = Y
            #cte_sum += Y
            turtle.left(od * 180 / pi)
            turtle.forward(distance2)

    def test_robot_treadmill(self):
        turn_degrees = 2
        param1, param2, param3 = 0., 3.0, 0.004

        max_speed = 10
        min_speed = 0

        _time = 0

        def get_time():
            return _time

        def get_distance_to_center_target():
            return center_target[1] - turtle.position()[1]

        def get_distance_to_left_target():
            return dist(left_target, turtle.position())

        def get_distance_to_right_target():
            return dist(right_target, turtle.position())

        myrobot = robot()
#         myrobot.set_noise(0.01, 0.01)
#         myrobot.set_steering_drift(0.001)
        myrobot.set(new_x=0.0, new_y=0.0, new_orientation=pi/2.)
        turtle.left(myrobot.orientation * 180 / pi)
        speed = 0.0 # motion distance is equal to speed (we assume time = 1)
        treadmill_speed = (0, -1.0)
        N = 300

        target_offset = 5
        left_target = rotate(0, target_offset, 30 * pi / 180.)
        center_target = (0, target_offset)
        right_target = rotate(0, target_offset, -30 * pi / 180.)

        speedpid = PID.PID(
            P=-0.2,
            I=0.01,
            D=0.004,
            get_time=get_time)
        speedpid.set_target(target_offset)

        anglepid = PID.PID(
            P=0.2,
            I=0.01,
            D=0.004,
            get_time=get_time)
        anglepid.set_target(0)

#         tau_p = param1
#         tau_d = param2
#         tau_i = param3
        alpha = 0
        distance_error = get_distance_to_center_target()
        for step in xrange(N):
            _time += 1
            #alpha = -tau_p * Y - tau_d * (Y - Y_last) - tau_i * cte_sum
            #+alpha=left, -alpha=right
            #alpha = turn_degrees * pi / 180.

            # +=too close, -=too far away
            #if distance_error
            speed = speedpid.get_output()
            speed2 = min(max(speed, min_speed), max_speed)
            print('speed:', speed)
            print('speed2:', speed2)

            alpha = 0

            myrobot, (distance2, od) = myrobot.move(steering=alpha, distance=speed2)

            if treadmill_speed:
                tx, ty = turtle.position()
                tx += treadmill_speed[0]
                ty += treadmill_speed[1]
                myrobot.x += treadmill_speed[0]
                myrobot.y += treadmill_speed[1]
                turtle.setposition(tx, ty)

            print(myrobot, 'alpha:', alpha, 'speed:', speed, 'distance_error:', distance_error)
            #Y_last = Y
            #cte_sum += Y
            turtle.left(od * 180 / pi)
            turtle.forward(distance2)

            distance_error = get_distance_to_center_target()
            print('distance_error:', distance_error)

            speedpid.update(distance_error)

if __name__ == '__main__':
    unittest.main()
