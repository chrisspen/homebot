#!/usr/bin/env python
"""
Initiates the spin head action.
"""

import roslib
#roslib.load_manifest('ros_homebot')
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the spin_head action, including the
# goal message and the result message.
import ros_homebot.msg

def spin_head_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (spin_headAction) to the constructor.
    client = actionlib.SimpleActionClient('spin_head', ros_homebot.msg.SpinHeadAction)

    # Waits until the action server has started up and started
    # listening for goals.
    print 'waiting for server...'
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = ros_homebot.msg.SpinHeadGoal()

    # Sends the goal to the action server.
    print 'sending goal'
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    print 'waiting for result...'
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A SpinHeadResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('spin_head_client_py')
        result = spin_head_client()
        print "Result:", result
    except rospy.ROSInterruptException:
        print "program interrupted before completion"