#! /usr/bin/env python
import time

import roslib
#roslib.load_manifest('ros_homebot')
import rospy
import actionlib

import ros_homebot.msg
from ros_homebot_python import constants as c
from ros_homebot_python.node import (
    subscribe_to_topic,
    get_topic_name,
    packet_to_service_type,
    packet_to_service_request_type,
)

class SpinHeadServer:
    """
    Spins the head around in short increments indefinitely.
    """
    _feedback = ros_homebot.msg.SpinHeadFeedback()
    _result = ros_homebot.msg.SpinHeadResult()
  
    def __init__(self, name):
        
        subscribe_to_topic(c.HEAD, c.ID_PAN_ANGLE, self.on_pan_angle_update)
        
        print 'Starting server...'
        self.server = actionlib.SimpleActionServer(
            name,
            ros_homebot.msg.SpinHeadAction,
            self.execute_spin_head,
            False)
        self.server.start()
        print 'Ready!'
        
        # How much we increment the pan angle to every N seconds.
        self.pan_angle_increment = 360/10
        
        # The current pan angle, as most recently updated.
        self.pan_angle = 0
        
        self.request_pan_angle_update()
        
    def request_pan_angle_update(self):
        service_name = get_topic_name(c.HEAD, c.ID_GET_VALUE)
        service_type = packet_to_service_type(c.ID_GET_VALUE)
#         service_req_type = packet_to_service_request_type(c.ID_GET_VALUE)
        rospy.ServiceProxy(service_name, service_type)(c.ID_PAN_ANGLE)

    def on_pan_angle_update(self, msg):
        print 'received pan angle update:', msg.angle
        self.pan_angle = msg.angle

    def set_pan_angle(self, angle, timeout=30):
        print 'setting pan angle to %s...' % angle
        t0 = time.time()
        while time.time() - t0 < timeout:
            
            # Call service to set angle.
            service_name = get_topic_name(c.HEAD, c.ID_PAN_ANGLE)
            service_type = packet_to_service_type(c.ID_PAN_ANGLE)
            rospy.ServiceProxy(service_name, service_type)(angle)
            
            # Wait for response.
            for _ in range(10):
                time.sleep(.1)
                if abs(self.pan_angle - angle) < 3:
                    print 'achieved!'
                    return

    def execute_spin_head(self, goal):
        
        desired_angle = self.pan_angle
        
        while not self.server.is_preempt_requested():
            print 'Spinning head...'
            
            # Increment angle.
            desired_angle += self.pan_angle_increment
            desired_angle %= 360
            self.set_pan_angle(desired_angle)
            
            # Wait.
            time.sleep(1)
            
        #self._as.publish_feedback(self._feedback)
        self.server.set_succeeded(self._result)


if __name__ == '__main__':
  rospy.init_node('spin_head')
  server = SpinHeadServer(rospy.get_name())
  rospy.spin()
  