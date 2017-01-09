#!/usr/bin/env python
import time
import threading
from math import pi, sin, cos

#import numpy as np
import rospy
import tf
import tf2_ros
#http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from std_srvs.srv import Empty, EmptyResponse
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Point
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from ros_homebot_python import constants as c
from ros_homebot_python.node import BaseArduinoNode

OK = DiagnosticStatus.OK
WARN = DiagnosticStatus.WARN
ERROR = DiagnosticStatus.ERROR
STALE = DiagnosticStatus.STALE

# Pololu 2282 Gear Motor => 464.64 counts per revolution of the gearbox's output shaft
# Driver wheel radius = 14 mm
# Tread length = 228 mm
#(revolution_of_shaft/counts) * (wheel_diameter)/(revolution_of_shaft)
#(revolution_of_shaft/464.6 counts) * (2*pi*14 mm)/(1 revolution_of_shaft) * (1m/1000mm)
#DistancePerCount = (3.14159265 * 0.1524) / 64000 * (1/1000.)
#TODO:the 464.6 counts may mean for quadrature, but we're only using a single channel
# Note, ROS distance assumes meters.
DistancePerCount = (3.141592653589793 * 28) / 464.6 / 1000.

# Based on https://goo.gl/mY0th1
# http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20broadcaster%20%28Python%29
# http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
# http://answers.ros.org/question/79851/python-odometry/
class WheelTracker(object):
  
    frame_id = '/odom'
    
    child_frame_id = '/base_link'
    
    # Covariance
    #P = np.mat(np.diag([0.0]*3))
  
    def __init__(self):
        
        self._lock = threading.RLock()
        
        self._reset_odometry()
        
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        
        # http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29
        self.tf_br = tf.TransformBroadcaster()
        #self.tf_br = tf2_ros.TransformBroadcaster()
        
        rospy.Service('~reset_odometry', Empty, self.reset_odometry)
    
    def _reset_odometry(self):
        
        self._PreviousLeftEncoderCounts = None
        self._PreviousRightEncoderCounts = None
        
        self.x = 0.
        self.y = 0.
        self.z = 0.
        self.th = 0.
        
        self.vx = 0.
        self.vy = 0.
        self.vz = 0.
        self.vth = 0.
        
        self.deltaLeft = 0
        self.deltaRight = 0
        
        self.last_time = None
    
    def reset_odometry(self, msg=None):
        with self._lock:
            self._reset_odometry()
            self.update()
            time.sleep(0.10)
            self.update()
            
        return EmptyResponse()
    
    def update_left(self, count):
        """
        Called when the left encoder generates a tick.
        """
        with self._lock:
            if self._PreviousLeftEncoderCounts is not None:
                self.deltaLeft = count - self._PreviousLeftEncoderCounts
                self.vx = self.deltaLeft * DistancePerCount
                self.update()
            self._PreviousLeftEncoderCounts = count
        
    def update_right(self, count):
        """
        Called when the right encoder generates a tick.
        """
        with self._lock:
            if self._PreviousRightEncoderCounts is not None:
                self.deltaRight = count - self._PreviousRightEncoderCounts
                self.vy = self.deltaRight * DistancePerCount
                self.update()
            self._PreviousRightEncoderCounts = count
        
    def update(self):
        
        self.current_time = rospy.Time.now()
        if self.last_time is not None:
            
            # compute odometry in a typical way given the velocities of the robot
            dt = (self.current_time - self.last_time).to_sec()
            delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * dt
            delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * dt
            delta_th = self.vth * dt
            self.x += delta_x
            self.y += delta_y
            self.th += delta_th
        
            # since all odometry is 6DOF we'll need a quaternion created from yaw
            #geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
            odom_quat = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, self.th))
        
            # next, we'll publish the odometry message over ROS
            #nav_msgs::Odometry odom
            msg = Odometry()
            msg.header.stamp = self.current_time
            msg.header.frame_id = self.frame_id
            msg.child_frame_id = self.child_frame_id
        
            # set the position
            msg.pose.pose.position = Point(self.x, self.y, self.z)
            msg.pose.pose.orientation = odom_quat
            
            # position covariance
#             p_cov[0:2,0:2] = self.P[0:2,0:2]
#             # orientation covariance for Yaw
#             # x and Yaw
#             p_cov[5,0] = p_cov[0,5] = self.P[2,0]
#             # y and Yaw
#             p_cov[5,1] = p_cov[1,5] = self.P[2,1]
#             # Yaw and Yaw
#             p_cov[5,5] = self.P[2,2]
#             msg.pose.covariance = tuple(p_cov.ravel().tolist())

            # set the velocity
            msg.twist.twist.linear.x = self.vx
            msg.twist.twist.linear.y = self.vy
            msg.twist.twist.angular.z = self.vth
        
            # publish the odometry message
            self.odom_pub.publish(msg)

            pos = (msg.pose.pose.position.x,
                   msg.pose.pose.position.y,
                   msg.pose.pose.position.z)
            
            ori = (msg.pose.pose.orientation.x,
                   msg.pose.pose.orientation.y,
                   msg.pose.pose.orientation.z,
                   msg.pose.pose.orientation.w)

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
#             self.tf_br.sendTransform(odom_trans) # for tf2
            self.tf_br.sendTransform(pos, ori, msg.header.stamp, msg.child_frame_id, msg.header.frame_id)
    
        self.last_time = self.current_time
        
        
class TorsoNode(BaseArduinoNode):
    
    name = c.NAME_TORSO
    
    last_accelerometer = None
    
    last_euler = None
    
    received_imu = False

    def create_publishers(self):
        self.imu_pub = rospy.Publisher('imu/data_raw', Imu, queue_size=01)
        
        self._imu_pub_lock = threading.RLock()
        self._imu_pub_thread = threading.Thread(target=self.publish_imu_thread)
        self._imu_pub_thread.daemon = True
        self._imu_pub_thread.start()
        
        self.wheel_tracker = WheelTracker()

    def _on_packet_motor_encoder(self, packet):
        """
        Tracks and re-publishes the wheel encoders.
        
            channel: 0|1
            count: +/-
        """
        try:
            packet_dict = self.get_packet_dict(packet)
            if not packet_dict:
                return
        except (ValueError, TypeError) as e:
            return
            
        channel = packet_dict['channel']
        count = packet_dict['count']
        if channel == 0:
            self.wheel_tracker.update_left(count)
        else:
            self.wheel_tracker.update_right(count)

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
    #speed = 38400
    speed = 115200
    TorsoNode(speed=speed)
