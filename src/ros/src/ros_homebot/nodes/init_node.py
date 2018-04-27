#!/usr/bin/env python

import time
from commands import getoutput

import rospy

import ros_homebot_msgs.srv
from ros_homebot_python import constants as c

from .common import publish

class InitROS():
    """
    Handles all post-startup init that can't be easily done via roslaunch.
    """
    def __init__(self):

        rospy.init_node('HomebotInit', log_level=rospy.DEBUG)

        # By default, torso does not publish much debugging info, so conserve USB bandwidth and processing resources.
        # Turn it on to aid in debugging.
        while not getoutput('rosnode list | grep /torso_arduino/debug_level'):
            print('Waiting for torso node to start...')
            time.sleep(1)
        publish('/torso_arduino/debug_level', 1) # std_msgs/Int16

        # Say something to let the world know we're ready.
        say = rospy.ServiceProxy('/sound/say', ros_homebot_msgs.srv.TTS)
        while not rospy.is_shutdown():
            try:
                say(c.SYSTEM_STARTUP_SPEECH)
                break
            except rospy.service.ServiceException as e:
                print e
                time.sleep(1)

if __name__ == '__main__':
    InitROS()
