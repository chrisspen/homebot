#!/usr/bin/env python

import time

import rospy
import std_srvs.srv

import ros_homebot_msgs.srv

class InitROS():
    """
    Handles all post-startup init that can't be easily done via roslaunch.
    """
    def __init__(self):
        
        rospy.init_node('HomebotInit', log_level=rospy.DEBUG)
        
        # Say something.
        say = rospy.ServiceProxy('/sound/say', ros_homebot_msgs.srv.TTS)
        while not rospy.is_shutdown():
            try:
                say('System initialized.', '', 0, 0)
                break
            except rospy.service.ServiceException as e:
                print e
                time.sleep(1)

if __name__ == '__main__':
    InitROS()
