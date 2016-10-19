#! /usr/bin/env python
import time
import random
import threading

import roslib
#roslib.load_manifest('ros_homebot')
import rospy
import actionlib
# from std_srvs.srv import Empty

import ros_homebot.msg
from ros_homebot_python import constants as c
from ros_homebot_python.node import (
    subscribe_to_topic,
    get_topic_name,
    packet_to_service_type,
    packet_to_service_request_type,
)
from ros_qr_tracker.msg import Percept
from ros_qr_tracker.srv import AddTarget, AddTargetResponse, SetTarget, SetTargetResponse

class Stages:
    SEARCHING = 1
    TRACKING = 2

class TrackQR:
    """
    Moves the head to track a QR code.
    """
  
    def __init__(self):
        
        rospy.init_node('track_qr')
        
        # Cleanup when termniating the node
        rospy.on_shutdown(self.shutdown)
        
        self.target = rospy.get_param("~target", '')
        assert self.target, 'No target specified.'
        
        self.rate = rospy.Rate(int(rospy.get_param("~rate", 60)))
        
        self._lock = RLock()
        
        # The last QR code match received.
        self.last_match_msg = None
        self.last_match_time = None
        
        # How much we increment the pan angle to every N seconds.
        self.pan_angle_increment = 360/10
        
        # The current pan angle, as most recently updated.
        self.pan_angle = 0
        
        self.tilt_angle = 0
        
        self.request_pan_angle_update()
        
        self.stage = Stages.SEARCHING
        
        self.subscribe()
        
        self.set_qr_target(self.target)
        
        self.execute()
    
    def subscribe(self):
        
        subscribe_to_topic(c.HEAD, c.ID_PAN_ANGLE, self.on_pan_angle_update)
        
        subscribe_to_topic(c.HEAD, c.ID_TILT_ANGLE, self.on_tilt_angle_update)
        
        rospy.Subscriber('/qr_tracker/matches', Percept, self.on_qr_match)
    
    def request_pan_angle_update(self):
        service_name = get_topic_name(c.HEAD, c.ID_GET_VALUE)
        service_type = packet_to_service_type(c.ID_GET_VALUE)
#         service_req_type = packet_to_service_request_type(c.ID_GET_VALUE)
        rospy.ServiceProxy(service_name, service_type)(c.ID_PAN_ANGLE)

    def on_qr_match(self, msg):
        with self._lock:
            self.last_match = msg
            self.last_match_time = time.time()

    def set_qr_target(self, target):
        rospy.ServiceProxy('/qr_tracker/clear_target')()
        rospy.ServiceProxy('/qr_tracker/set_target', SetTarget)(target)
        rospy.ServiceProxy('/qr_tracker/start')()

    def on_pan_angle_update(self, msg):
        print 'received pan angle update:', msg.angle
        self.pan_angle = msg.angle

    def on_tilt_angle_update(self, msg):
        print 'received tilt angle update:', msg.angle
        self.tilt_angle = msg.angle

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

    def say(self, text):
        say = rospy.ServiceProxy('/sound/say', ros_homebot_msgs.srv.TTS)
        say(text, '', 0, 0)

    def execute(self, goal):
        while not rospy.is_shutdown():
            print('last match:', self.last_match)
                
            if self.stage == Stages.SEARCHING:
                
                # Slowly spin head 360 until QR found.
                if self.pan_angle is not None:
                    self.pan_angle += 1
                    self.pan_angle = self.pan_angle % 360
                    self.set_pan_angle(self.pan_angle)
                else:
                    print('no pan_angle')
                
                if self.last_match_time and (time.time() - self.last_match_time) < 1:
                    self.say('Target found. Tracking engaged.')
                    self.stage = Stages.TRACKING
                else:
                    print('no last_match_time')
                
            elif self.stage == Stages.TRACKING:
                
                # Find QR centerpoint.
                
                # Move pan to center horizontally.
                
                # Move tilt to center vertically.
                
                if self.last_match_time and (time.time() - self.last_match_time) > 5:
                    self.stage = Stages.SEARCHING
                    self.say('Target lost. Initiating search.')
                
    #         desired_angle = self.pan_angle
    #         
    #         while not self.server.is_preempt_requested():
    #             print 'Spinning head...'
    #             
    #             # Increment angle.
    #             desired_angle += self.pan_angle_increment
    #             desired_angle %= 360
    #             self.set_pan_angle(desired_angle)
    #             
    #             # Wait.
    #             time.sleep(1)
    #             
    #         #self._as.publish_feedback(self._feedback)
    #         self.server.set_succeeded(self._result)
            self.rate.sleep()

    def shutdown(self):
        rospy.loginfo('Shutting down...')
        rospy.ServiceProxy('/qr_tracker/stop')()
        rospy.loginfo('Done.')
        
if __name__ == '__main__':
  TrackQR()
