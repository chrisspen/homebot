#!/usr/bin/env python
"""
given a point relative to the baseframe, calculate the necessary pan+tilt angle to point camera at it
given a pan+tilt angle, calculate the point relative to the base frame
"""

import unittest

#https://github.com/ros/geometry/blob/hydro-devel/tf/src/tf/transformations.py
from tf import transformations as tf
import numpy as np

from ros_homebot_python import constants as c
from ros_homebot_python import utils

class Tests(unittest.TestCase):

    def test_transformations(self):
        from math import pi
        alpha, beta, gamma = 0, 0, -pi/2.
        origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
#         Rx = tf.rotation_matrix(alpha, xaxis)
#         Ry = tf.rotation_matrix(beta, yaxis)
        R = Rz = tf.rotation_matrix(gamma, zaxis)
        #R = tf.concatenate_matrices(Rx, Ry, Rz)

        point0 = np.array([0, 1, 0, 0])
        print point0

        #point1 = R * point0
        #point1 = R * point0.T
        #point1 = np.multiply(R, point0)
        #point1 = np.multiply(R, point0.T)
        actual_point1 = R.dot(point0)
        print 'actual_point1:', actual_point1

        expected_point1 = np.array([1, 0, 0, 0])
        print 'expected_point1:', expected_point1

        #np.alltrue(point1 == )
        #self.assertEqual(point1.tolist(), expected_point1.tolist())
        self.assertTrue(np.allclose(actual_point1, expected_point1))

    def test_translate(self):

        p0 = np.array((10, 0, 1, 0))
        #p1_actual = tf.translation_matrix((0, 0, -1, 10)).dot(p0)
        p1_actual = np.add((0, 0, -1, 0), p0)
        p1_expected = np.array((10, 0, 0, 0))
        print 'p1_expected:', p1_expected
        print 'p1_actual:', p1_actual
        self.assertTrue(np.allclose(p1_actual, p1_expected))

    def test_head_angles(self):

        test_data = [
            #((pan angle, tilt angle, distance), (expected point))
            (
                # looking straight ahead
                (0, 90, c.TORSO_DIAMETER_MM*2),
                (c.TORSO_DIAMETER_MM+c.TORSO_DIAMETER_MM*2, 0, c.HEIGHT_CENTER_HEIGHT_MM),
            ),
            (
                # tilt head down max
                (0, c.TILT_MIN, c.TORSO_DIAMETER_MM*2),
                (389.7114317029974, 0.0, 10.000000000000028),
            ),
            (
                # tilt head up max
                (0, c.TILT_MAX, c.TORSO_DIAMETER_MM*2),
                (389.7114317029974, 0.0, 460),
            ),
            (
                # tilt head down max and 90-degrees to the right
                (90, c.TILT_MIN, c.TORSO_DIAMETER_MM*2),
                (0, 389.7114317029974, 10.000000000000028),
            ),
        ]

        for (pan_angle, tilt_angle, distance), expected_point in test_data:
            print 'pan=%s, tilt=%s, distance=%s' % (pan_angle, tilt_angle, distance)
            actual_point = utils.head_angles_to_point(pan=pan_angle, tilt=tilt_angle, distance=distance)
            print 'expected_point:'
            print expected_point
            print 'actual_point:'
            print actual_point
            self.assertTrue(np.allclose(actual_point, expected_point))

        test_data = [
            ((c.TORSO_DIAMETER_MM*3, 0, 0), (0, 0)),
        ]

        # for point, expected_angles in test_data:
        #     angles = utils.point_to_angles(point)
        #     print angles, expected_angles
        #     assert angles == expected_angles

if __name__ == '__main__':
    unittest.main()
