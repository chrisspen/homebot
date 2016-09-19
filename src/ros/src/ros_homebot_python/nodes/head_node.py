#!/usr/bin/env python
from math import pi

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from ros_homebot_python import constants as c
from ros_homebot_python.node import BaseArduinoNode

class HeadNode(BaseArduinoNode):
    
    name = c.NAME_HEAD
    
    def create_publishers(self):
        self.joint_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        
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
          
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = [c.TORSO_TO_NECK_JOINT]
        joint_state.position = [packet_dict['angle']*pi/180.]
        joint_state.velocity = []
        joint_state.effort = []
        self.joint_pub.publish(joint_state)
        
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
            
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = [c.NECK_TO_HEAD_JOINT]
        joint_state.position = [packet_dict['angle']*pi/180.]
        joint_state.velocity = []
        joint_state.effort = []
        self.joint_pub.publish(joint_state)
           
if __name__ == '__main__':
    HeadNode()
