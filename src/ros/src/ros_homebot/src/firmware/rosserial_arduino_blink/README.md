Overview
--------

An example usage of the Rosserial Arduino blink code.

http://wiki.ros.org/rosserial_arduino/Tutorials/Blink

Note, rosserial has considerable overhead and consumes almost 30% of the Arduino Uno's memory.

Installation
------------

http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup#Installing_from_Source_onto_the_ROS_workstation
    
    ROS_DISTRO=kinetic
    source /opt/ros/$ROS_DISTRO/setup.bash
    sudo apt-get install ros-$ROS_DISTRO-rosserial-arduino ros-$ROS_DISTRO-rosserial arduino-mk
    cd ~/Arduino/libraries
    rm -rf ros_lib
    rosrun rosserial_arduino make_libraries.py .
    cd <project_cwd>
    make
    make upload

Usage
-----

Run:

    roscore
    rosrun rosserial_python serial_node.py /dev/ttyACM0
    rostopic pub toggle_led std_msgs/Empty --once
    
To make the LED blink twice a second, run:

    rostopic pub toggle_led std_msgs/Empty -r 2
