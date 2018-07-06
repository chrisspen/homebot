#!/bin/bash
export ROS_MASTER_URI=http://rae.local:11311
rostopic pub --once /torso_arduino/undock std_msgs/Empty
sleep 5
rosrun ros_homebot dock.py --audible
