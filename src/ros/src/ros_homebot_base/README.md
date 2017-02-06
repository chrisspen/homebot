Overview
--------

Implements the base firmware and ROS node for communicating with the torso Arduino.

Development
-----------

Install the necessary system packages:

    sudo apt-get install ros-$ROS_DISTRO-serial

To build the ROS node, run:

    cd <package_cwd>../..
    time catkin_make clean
    time catkin_make --pkg ros_homebot_base

To confirm the nodes were compiled and installed correctly, run:

    cd <package_cwd>../..
    rospack list|grep -i homebot_base    

To build the Arduino firmware:

    cd <package_cwd>src/firmware/echo
    make

Running
-------

Upload the firmware to the torso Arduino, run:

    cd <package_cwd>src/firmware/echo
    make upload
    
To test the node, run:

    roscore
    rosrun ros_homebot_base serial_echo_node

Then start up two separate ROS shells.

In one run:

    rostopic echo /read

And in another call:

    rostopic pub --once /write std_msgs/String hello

Assuming the Arduino is connected and running, in the read shell, you should see "hello" printed.

To stress-test the connection, you can repeatedly write at a specific rate with:

    rostopic pub -r 60 /write std_msgs/String hello
