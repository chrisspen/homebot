#!/usr/bin/env python
"""
Initiates the wandering action.
"""
import traceback, time

import roslib
#roslib.load_manifest('ros_homebot')
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the wander action, including the
# goal message and the result message.
import ros_homebot.msg

class Client:
    
    def __init__(self):
        
        rospy.init_node('wander_client', log_level=rospy.DEBUG)
        
        # Creates the SimpleActionClient, passing the type of the action
        # (spin_headAction) to the constructor.
        self.client = actionlib.SimpleActionClient('wander', ros_homebot.msg.WanderAction)
    
        rospy.on_shutdown(self.shutdown)
        
        try:
            # Waits until the action server has started up and started
            # listening for goals.
            print 'waiting for server...'
            self.client.wait_for_server()
            self.client.cancel_all_goals()
        
            # Creates a goal to send to the action server.
            goal = ros_homebot.msg.WanderGoal()
        
            # Sends the goal to the action server.
            print 'sending goal'
            self.client.send_goal(goal)
        
            # Waits for the server to finish performing the action.
            print 'waiting for result...'
            self.client.wait_for_result()
        
            # Prints out the result of executing the action
            self.client.get_result()  # A WanderResult
    
        except Exception as e:
            traceback.print_exc(file=sys.stderr)
            
    def shutdown(self):
        print 'Cancelling goal...'
        self.client.cancel_all_goals()
        time.sleep(1)
        print 'Done.'
        

if __name__ == '__main__':
    Client()
    