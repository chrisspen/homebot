#!/usr/bin/env python
from __future__ import print_function
#import time
import traceback
import os
import sys
import threading
import cPickle as pickle
from functools import partial
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

IMU_CALIBRATION_FN = 'imu_calibration.pickle'

V0 = 'v0'
V1 = 'v1'

status_id_to_name = {
    OK: 'OK',
    WARN: 'WARN',
    ERROR: 'ERROR',
    STALE: 'STALE',
}

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
class ArduinoRelay:

    cache_dir = '~/.homebot_cache/torso_relay'

    # Covariance
    #P = np.mat(np.diag([0.0]*3))

    def __init__(self):
        rospy.init_node('arduino_relay')

        self._imu_data = {}

        self._lock = threading.RLock()

        self.imu_calibration_loaded = False

        self.diagnostics_msg_count = 0

        self.diagnostics_buffer = []

        # http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29
        self.tf_br = tf.TransformBroadcaster()
        #self.tf2_br = tf2_ros.TransformBroadcaster()

        #rospy.Service('~reset_odometry', Empty, self.reset_odometry)

        ## Publishers.

        self.diagnostics_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)

        self.odometry_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

        self.imu_calibration_load_pub = rospy.Publisher('/torso_arduino/imu/calibration/load', UInt16MultiArray, queue_size=1)

        self.imu_pub = rospy.Publisher('imu/data_raw', Imu, queue_size=10)

        ## Subscribers.

        rospy.Subscriber('/head_arduino/diagnostics_relay', String, partial(self.on_diagnostics_relay, prefix='head'))
        rospy.Subscriber('/torso_arduino/diagnostics_relay', String, partial(self.on_diagnostics_relay, prefix='torso'))

        rospy.Subscriber('/torso_arduino/imu/calibration/save', UInt16MultiArray, self.on_imu_calibration_save)

        rospy.Subscriber('/torso_arduino/imu_relay', String, self.on_imu_relay)

        rospy.Subscriber('/torso_arduino/odometry_relay', String, self.on_odometry_relay)

        ## Begin IO.

        rospy.spin()

    @property
    def imu_calibration_filename(self):
        cache_dir = os.path.expanduser(self.cache_dir)
        if not os.path.isdir(cache_dir):
            os.makedirs(cache_dir)
        fn = os.path.join(cache_dir, IMU_CALIBRATION_FN)
        return fn

    def load_imu_calibration(self):
        """
        Automatically called once after the first diagnostic message is received.

        Per Adafruit's documentation:

        "One thing to keep in mind though is that the sensor isn't necessarily 'plug and play' with
        loading the calibration data, in particular the magnetometer needs to be recalibrated even if
        the offsets are loaded. The magnetometer calibration is very dynamic so saving the values
        once might"
        """
        try:
            fn = self.imu_calibration_filename
            if os.path.isfile(fn):
                print('Loading imu calibration %s...' % fn)
                with open(fn, 'r') as fin:
                    msg = pickle.load(fin)
                print('Sending calibration:')
                print(msg)
                self.imu_calibration_load_pub.publish(msg)
            else:
                print('No saved imu calibration.')
        except Exception as exc:
            traceback.print_exc()
        finally:
            self.imu_calibration_loaded = True

    def on_imu_calibration_save(self, msg):
        #print('Received imu calibration:', msg)
        if sum(msg.data) == 0:
            print('Ignoring blank calibration.')
            self.load_imu_calibration()
            return
        fn = self.imu_calibration_filename
        with open(fn, 'w') as fout:
            pickle.dump(msg, fout)

    def on_imu_relay(self, msg):
        #print('imu_relay.msg:', msg)
        parts = msg.data.split(':')

        # Validate type.
        typ = parts[0]
        assert typ in 'aeg', 'Invalid typ: %s' % typ

        # Conver the integers to the original floats.
        nums = ltof(parts[1:])
        for num, axis in zip(nums, 'xyz'):
            self._imu_data['%s%s' % (typ, axis)] = num
        #print('imu_data:', self._imu_data)

        # If we've received the final segment, re-publish the complete IMU message.
        if typ == 'a':
            imu_msg = Imu()
            imu_msg.header = Header()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = c.BASE_LINK

            # Our sensor returns Euler angles in degrees, but ROS requires radians.
            # http://answers.ros.org/question/69754/quaternion-transformations-in-python/
            roll = self._imu_data['ex']
            pitch = self._imu_data['ey']
            yaw = self._imu_data['ez']
            quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            imu_msg.orientation.x = quaternion[0]
            imu_msg.orientation.y = quaternion[1]
            imu_msg.orientation.z = quaternion[2]
            imu_msg.orientation.w = quaternion[3]

            imu_msg.angular_velocity.x = self._imu_data['gx']
            imu_msg.angular_velocity.y = self._imu_data['gy']
            imu_msg.angular_velocity.z = self._imu_data['gz']

            imu_msg.linear_acceleration.x = self._imu_data['ax']
            imu_msg.linear_acceleration.y = self._imu_data['ay']
            imu_msg.linear_acceleration.z = self._imu_data['az']

            self.imu_pub.publish(imu_msg)

    def on_diagnostics_relay(self, msg, prefix):
        """
        The Arduino has limited RAM and an even more limited serial buffer, so it can't send complex ROS structures like DiagnosticArrays.
        So instead, it publishes diagnostic data via a key/value pair formatted in a simple string,
        which we convert to a proper diagnostic message.
        """
        #print('diagnostics.msg:', msg)
        self.diagnostics_msg_count += 1

        if not self.imu_calibration_loaded:
            self.load_imu_calibration()
            self.imu_calibration_loaded = True

        # Aggregate single-character messages into a complete message.
        #if not msg.data:
            #print('Received empty message.')
            #return
        #elif msg.data[0] == '^':
            #print('Received message start.')
            #self.diagnostics_buffer = []
            #return
        #elif msg.data[0] == '$':
            #print('Received message end.')
            #msg.data = ''.join(self.diagnostics_buffer)
        #else:
            #print('Recieved %i chars.' % len(msg.data))
            #self.diagnostics_buffer.append(msg.data)
            #return

        # Extract parts.
        print('Message length:', len(msg.data))
        print('Message data:', msg.data)
        parts = msg.data.split(':')
        if len(parts) < 2:
            print('Malformed diagnostics message.', file=sys.stderr)
            return

        # Complete name part.
        name = '%s: %s' % (prefix, parts[0].strip())

        # Complete level part.
        try:
            level = int(parts[1]) # OK|WARN|ERROR|STALE
            assert level in range(4)
        except (TypeError, ValueError, AssertionError) as exc:
            print('Malformed level: %s' % parts[1])

        # Complete message part.
        message = ''
        if len(parts) >= 3:
            message = parts[2].strip()
            if message == '?':
                message = ''
        if not message:
            # If not given, default the message to the name equivalent of the level.
            message = status_id_to_name.get(level, '')

        # Construct and send diagnostics array.
        # http://docs.ros.org/api/diagnostic_msgs/html/msg/DiagnosticStatus.html
        array = DiagnosticArray()
        array.status = [
            DiagnosticStatus(name=name, level=level, message=message)
        ]
        self.diagnostics_pub.publish(array)

    def on_odometry_relay(self, msg):

        print('odometry.msg:', msg)

        parts = msg.data.split(':')
        if len(parts) < 5:
            print('Malformed odometry message.', file=sys.stderr)
            return

        # Validate type.
        typ = parts[0]
        assert typ in (V0, V1), 'Invalid type: %s' % typ

        # Validate numbers.
        nums = ltof(parts[1:])

        # Save type parts.
        if typ == V0:
            # Position.
            # x,y,z,th
            self._odometry_v0 = nums
        else:
            # Velocity.
            # vx,vy,vz,vth
            self._odometry_v1 = nums

        # Combine and publish a complete odometry message on the receipt of the last part.
        if typ == V1:
            current_time = rospy.Time.now()
            print('position:', self._odometry_v0)
            x, y, z, th = self._odometry_v0
            print('velocity:', self._odometry_v1)
            vx, vy, vz, vth = self._odometry_v1

            # since all odometry is 6DOF we'll need a quaternion created from yaw
            #geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
            odom_quat = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, th))

            msg = Odometry()
            msg.header.stamp = current_time
            msg.header.frame_id = c.ODOM
            msg.child_frame_id = c.BASE_LINK
            msg.pose.pose.position = Point(x, y, z)
            msg.pose.pose.orientation = odom_quat
            msg.twist.twist.linear.x = vx
            msg.twist.twist.linear.y = vy
            msg.twist.twist.angular.z = vth

            # publish the odometry message
            self.odometry_pub.publish(msg)

            pos = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            )

            ori = (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            )

            # first, we'll publish the transform over tf
            #geometry_msgs::TransformStamped odom_trans;
#             odom_trans = TransformStamped()
#             odom_trans.header.stamp = self.current_time
#             odom_trans.header.frame_id = self.frame_id
#             odom_trans.child_frame_id = self.child_frame_id
#             odom_trans.transform.translation.x = self.x
#             odom_trans.transform.translation.y = self.y
#             odom_trans.transform.translation.z = self.z
#             odom_trans.transform.rotation = odom_quat

            # send the transform
            self.tf_br.sendTransform(pos, ori, msg.header.stamp, msg.child_frame_id, msg.header.frame_id)
            #self.tf2_br.sendTransform(odom_trans)

if __name__ == '__main__':
    ArduinoRelay()
