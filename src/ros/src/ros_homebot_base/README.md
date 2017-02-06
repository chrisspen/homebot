Overview
--------

Implements the base firmware and ROS node for communicating with the torso Arduino.

Development
-----------

Install the necessary system packages:

    sudo apt-get install ros-$ROS_DISTRO-serial

To build the ROS node, run:

    cd <package_cwd>../..
    time catkin_make --pkg ros_homebot_base

To confirm the nodes were compiled and installed correctly, run:

    cd <package_cwd>../..
    rospack list|grep -i homebot_base    

To test the node, run:

    roscore
    rosrun ros_homebot_base serial_example_node

To build and upload the firmware to the torso Arduino, run:

    make; make upload
