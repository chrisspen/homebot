#!/usr/bin/env python

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..'))

# PKG = 'ros_homebot_python'
# import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.

from ros_homebot_python import utils
from ros_homebot_python import constants as c

class Tests(unittest.TestCase):
            
    def test_relative_to_absolute_speed(self):
        
        try:
            utils.relative_to_absolute_speed(300)
            self.assertTrue(0)
        except AssertionError:
            pass
            
        self.assertEqual(
            utils.relative_to_absolute_speed(c.MOTOR_MAX_SPEED),
            c.MOTOR_MAX_SPEED_REAL)
            
        self.assertEqual(
            utils.relative_to_absolute_speed(-c.MOTOR_MAX_SPEED),
            -c.MOTOR_MAX_SPEED_REAL)
            
        self.assertEqual(
            utils.relative_to_absolute_speed(0),
            0 * c.MM/c.SEC)
    
    def test_linear_travel_time(self):
        
        try:
            utils.linear_travel_time(1, 2)
            self.assertTrue(0)
        except (AttributeError, TypeError, ValueError) as e:
            pass
            
        self.assertEqual(
            utils.linear_travel_time(speed=1*c.MM/c.SEC, distance=2*c.MM),
            2*c.SEC)
            
        self.assertEqual(
            utils.linear_travel_time(speed=-1*c.MM/c.SEC, distance=2*c.MM),
            2*c.SEC)
            
        self.assertAlmostEqual(
            utils.linear_travel_time(
                speed=1*c.MM/c.SEC,
                distance=10*c.MM,
                start_speed=0*c.MM/c.SEC,
                end_speed=0*c.MM/c.SEC,
            ).magnitude,
            10.0026740772)
            
    def test_rotational_travel_time(self):
        from math import pi
        
        C = (pi * c.TORSO_TREAD_WIDTH).magnitude
        
        self.assertEqual(
            utils.rotational_travel_time(speed=C*.25*c.MM/c.SEC, degrees=90*c.DEG),
            1*c.SEC)
            
        self.assertEqual(
            utils.rotational_travel_time(speed=C*.25*c.MM/c.SEC, degrees=-90*c.DEG),
            1*c.SEC)

if __name__ == '__main__':
#     import rosunit
#     rosunit.unitrun(PKG, 'test_name', 'tests.tests.Tests')
    
#     import rostest
#     rostest.rosrun(PKG, 'test_bare_bones', Tests)
    
    unittest.main()
    