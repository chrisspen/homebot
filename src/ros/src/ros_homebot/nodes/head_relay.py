#!/usr/bin/env python
from __future__ import print_function
#import time
import traceback
import os
import sys
import threading
import cPickle as pickle
#from math import pi, sin, cos

#import numpy as np
import rospy
import tf
#import tf2_ros
#http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from std_msgs.msg import String, UInt16MultiArray
#from std_srvs.srv import Empty, EmptyResponse
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus#, KeyValue

from ros_homebot_python import constants as c
#from ros_homebot_python.node import BaseArduinoNode

OK = DiagnosticStatus.OK # 0
WARN = DiagnosticStatus.WARN # 1
ERROR = DiagnosticStatus.ERROR # 2
STALE = DiagnosticStatus.STALE # 3

V0 = 'v0'
V1 = 'v1'

status_id_to_name = {
    OK: 'OK',
    WARN: 'WARN',
    ERROR: 'ERROR',
    STALE: 'STALE',
}

IMU_CALIBRATION_FN = 'imu_calibration.pickle'

def ltof(values):
    """
    Converts the special integer-encoded doubles back into Python floats.

    See the Arduino's equivalent ftol().
    """
    assert isinstance(values, (tuple, list))
    return [int(_)/1000. for _ in values]

# Based on https://goo.gl/mY0th1
# http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20broadcaster%20%28Python%29
# http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
# http://answers.ros.org/question/79851/python-odometry/
class HeadRelay:

    diagnostics_prefix = 'Head Arduino'

    cache_dir = '~/.homebot_cache/head_relay'

    def __init__(self):
        rospy.init_node('head_relay')

        self._imu_data = {}

        self._lock = threading.RLock()

        self.diagnostics_msg_count = 0

        #rospy.Service('~reset_odometry', Empty, self.reset_odometry)

        ## Publishers.

        self.diagnostics_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)

        ## Subscribers.

        rospy.Subscriber('/torso_arduino/diagnostics_relay', String, self.on_diagnostics_relay)

        ## Begin IO.

        rospy.spin()

    def on_diagnostics_relay(self, msg):
        """
        The Arduino has limited RAM and an even more limited serial buffer, so it can't send complex ROS structures like DiagnosticArrays.
        So instead, it publishes diagnostic data via a key/value pair formatted in a simple string,
        which we convert to a proper diagnostic message.
        """
        #print('diagnostics.msg:', msg)

if __name__ == '__main__':
    HeadRelay()
