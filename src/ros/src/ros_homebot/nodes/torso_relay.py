#!/usr/bin/env python
from __future__ import print_function
#import time
import traceback
import os
import threading
import cPickle as pickle
#from math import pi, sin, cos

#import numpy as np
import rospy
#import tf
# import tf2_ros
#http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
#from sensor_msgs.msg import Imu
from std_msgs.msg import String, UInt16MultiArray
#from std_srvs.srv import Empty, EmptyResponse
#from nav_msgs.msg import Odometry
#from geometry_msgs.msg import Quaternion, Point
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus#, KeyValue

#from ros_homebot_python import constants as c
#from ros_homebot_python.node import BaseArduinoNode

OK = DiagnosticStatus.OK # 0
WARN = DiagnosticStatus.WARN # 1
ERROR = DiagnosticStatus.ERROR # 2
STALE = DiagnosticStatus.STALE # 3

status_id_to_name = {
    OK: 'OK',
    WARN: 'WARN',
    ERROR: 'ERROR',
    STALE: 'STALE',
}

IMU_CALIBRATION_FN = 'imu_calibration.pickle'

# Based on https://goo.gl/mY0th1
# http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20broadcaster%20%28Python%29
# http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
# http://answers.ros.org/question/79851/python-odometry/
class TorsoRelay:
  
    #frame_id = '/odom'
    
    #child_frame_id = '/base_link'
    
    diagnostics_prefix = 'Torso Arduino'
    
    cache_dir = '~/.homebot_cache/torso_relay'
    
    # Covariance
    #P = np.mat(np.diag([0.0]*3))
  
    def __init__(self):
        rospy.init_node('torso_relay')
        
        self._lock = threading.RLock()
        
        self.imu_calibration_loaded = False
        
        self.diagnostics_msg_count = 0
        
        #self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        
        # http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29
        #self.tf_br = tf.TransformBroadcaster()
        #self.tf_br = tf2_ros.TransformBroadcaster()
        
        #rospy.Service('~reset_odometry', Empty, self.reset_odometry)

        self.diagnostics_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)
        
        self.imu_calibration_load_pub = rospy.Publisher('/torso_arduino/imu/calibration/load', UInt16MultiArray, queue_size=1)

        rospy.Subscriber('/torso_arduino/diagnostics_relay', String, self.on_diagnostics)
        
        rospy.Subscriber('/torso_arduino/imu/calibration/save', UInt16MultiArray, self.on_imu_calibration_save)
        
        rospy.spin()
    
    @property
    def imu_calibration_filename(self):
        cache_dir = self.cache_dir
        cache_dir = os.path.expanduser(cache_dir)
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
        print('Received imu calibration:', msg)
        if sum(msg.data) == 0:
            print('Ignoring blank calibration.')
            self.load_imu_calibration()
            return
        fn = self.imu_calibration_filename
        with open(fn, 'w') as fout:
            pickle.dump(msg, fout)

    def on_diagnostics(self, msg):
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
        
        # Extract parts.
        parts = msg.data.split(':')
        if len(parts) < 2:
            print('Malformed diagnostics message.')
            return
        
        # Complete name part.
        name = '%s: %s' % (self.diagnostics_prefix, parts[0].strip())
        
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

if __name__ == '__main__':
    TorsoRelay()
