#!/usr/bin/env python
import time
import rospy
rospy.init_node('test_node')
while 1:
    rospy.loginfo("Waiting for 1 second...")
    time.sleep(1)
