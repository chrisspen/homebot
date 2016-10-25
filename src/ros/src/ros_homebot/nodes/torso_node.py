#!/usr/bin/env python
import time
import threading
from math import pi

import rospy
import tf
#http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from ros_homebot_python import constants as c
from ros_homebot_python.node import BaseArduinoNode

OK = DiagnosticStatus.OK
WARN = DiagnosticStatus.WARN
ERROR = DiagnosticStatus.ERROR
STALE = DiagnosticStatus.STALE

class TorsoNode(BaseArduinoNode):
    
    name = c.NAME_TORSO
    
    last_accelerometer = None
    
    last_euler = None
    
    received_imu = False

    def create_publishers(self):
        self.imu_pub = rospy.Publisher('imu/data_raw', Imu, queue_size=1)
        
        self._imu_pub_lock = threading.RLock()
        
        self._imu_pub_thread = threading.Thread(target=self.publish_imu_thread)
        self._imu_pub_thread.daemon = True
        self._imu_pub_thread.start()

    def _on_packet_imu_accelerometer(self, packet):
        """
        Tracks and re-publishes the IMU orientation.
        
            device: 2
            x: 354.0
            y: -0.5
            z: 1.55999994278
        """
        
        try:
            packet_dict = self.get_packet_dict(packet)
            if not packet_dict:
                return
        except (ValueError, TypeError) as e:
            return
        
        self.last_accelerometer = packet_dict
        
        self.received_imu = True

    def _on_packet_imu_euler(self, packet):
        """
        Tracks and re-publishes the IMU orientation.
        
            device: 2
            x: 354.0
            y: -0.5
            z: 1.55999994278
        """
        
        try:
            packet_dict = self.get_packet_dict(packet)
            if not packet_dict:
                return
        except (ValueError, TypeError) as e:
            return
        
        self.last_euler = packet_dict
        
        self.received_imu = True
        
    def publish_imu_thread(self):
        time.sleep(3)
        while 1:
            if not self.received_imu:
                self.force_sensors()
            self.publish_imu()
            time.sleep(.1)
    
    def publish_imu(self):
        if not self.last_euler or not self.last_accelerometer:
            return
        with self._imu_pub_lock:
            imu_msg = Imu()
            imu_msg.header = Header()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = c.BASE_LINK
            
            # Our sensor returns Euler angles in degrees, but ROS requires radians.
            roll = self.last_euler['x'] * pi/180.
            pitch = self.last_euler['y'] * pi/180.
            yaw = self.last_euler['z'] * pi/180.
            # http://answers.ros.org/question/69754/quaternion-transformations-in-python/
            quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            imu_msg.orientation.x = quaternion[0]
            imu_msg.orientation.y = quaternion[1]
            imu_msg.orientation.z = quaternion[2]
            imu_msg.orientation.w = quaternion[3]
            
            #imu_msg.angular_velocity_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
#             imu_msg.angular_velocity.x = ns.omgx();
#             imu_msg.angular_velocity.y = ns.omgy();
#             imu_msg.angular_velocity.z = ns.omgz();
            #imu_msg.angular_velocity_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
            imu_msg.linear_acceleration.x = self.last_accelerometer['x']
            imu_msg.linear_acceleration.y = self.last_accelerometer['y']
            imu_msg.linear_acceleration.z = self.last_accelerometer['z']
            #imu_msg.angular_velocity_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};    
            self.imu_pub.publish(imu_msg)
           
if __name__ == '__main__':
    speed = 38400
    #speed = 115200
    TorsoNode(speed=speed)
