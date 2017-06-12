#!/usr/bin/env python
from math import pi
import threading
import time

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from ros_homebot_python import constants as c
from ros_homebot_python.node import BaseArduinoNode

class HeadNode(BaseArduinoNode):

    name = c.NAME_HEAD

    last_pan_angle = 0

    last_tilt_angle = 0

    received_angles = False

    def create_publishers(self):
        self.joint_pub = rospy.Publisher('joint_states', JointState, queue_size=10)

        self._joint_pub_lock = threading.RLock()

        self._joint_pub_thread = threading.Thread(target=self.publish_joints_thread)
        self._joint_pub_thread.daemon = True
        self._joint_pub_thread.start()

    def _on_packet_pan_angle(self, packet):
        """
        Re-publishes the pan angle as a standard JointState message.
        """

        try:
            packet_dict = self.get_packet_dict(packet)
            if not packet_dict:
                return
        except (ValueError, TypeError) as e:
            return

        self.last_pan_angle = packet_dict['angle']*pi/180.

        self.received_angles = True

        self.publish_joints()

    def _on_packet_tilt_angle(self, packet):
        """
        Re-publishes the tilt angle as a standard JointState message.
        """

        try:
            packet_dict = self.get_packet_dict(packet)
            if not packet_dict:
                return
        except (ValueError, TypeError) as e:
            return

        self.last_tilt_angle = packet_dict['angle']*pi/180.

        self.received_angles = True

        self.publish_joints()

    def publish_joints_thread(self):
        time.sleep(3)
        while 1:
            if not self.received_angles:
                self.force_sensors()
            self.publish_joints()
            time.sleep(1)

    def publish_joints(self):
        with self._joint_pub_lock:
            joint_state = JointState()
            joint_state.header = Header()
            joint_state.header.stamp = rospy.Time.now()
            joint_state.name = [
                c.FOOTPRINT_TO_TORSO_JOINT,
                c.TORSO_TO_NECK_JOINT,
                c.NECK_TO_HEAD_JOINT,
                c.HEAD_TO_CAMERA_JOINT,
            ]
            joint_state.position = [
                0,
                self.last_pan_angle,
                self.last_tilt_angle,
                0,
            ]
            joint_state.velocity = []
            joint_state.effort = []
            self.joint_pub.publish(joint_state)

if __name__ == '__main__':
    HeadNode()
