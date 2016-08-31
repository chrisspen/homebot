#!/usr/bin/env python
"""
Initiates the spin head action.
"""
import os, sys

import roslib
#roslib.load_manifest('ros_homebot')
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the spin_head action, including the
# goal message and the result message.
import ros_homebot.msg

from ros_homebot_python import constants as c

def say():
    # Creates the SimpleActionClient, passing the type of the action
    # (spin_headAction) to the constructor.
    client = actionlib.SimpleActionClient(c.SOUND_TTS, ros_homebot.msg.TTSAction)

    text = ' '.join(sys.argv[1:])
    print 'asking to say:', text
    
    # Waits until the action server has started up and started
    # listening for goals.
    print 'waiting for server...'
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = ros_homebot.msg.TTSGoal()
    goal.text = text

    # Sends the goal to the action server.
    print 'sending goal'
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    print 'waiting for result...'
    client.wait_for_result()
    
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('say')
        say()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"