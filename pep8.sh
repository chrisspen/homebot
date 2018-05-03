#!/bin/bash
pylint --rcfile=pylint.rc \
    satchels \
    src/ros/src/ros_homebot/nodes \
    src/ros/src/ros_homebot_lrf/nodes \
    src/ros/src/ros_homebot_lrf/src/ros_homebot_lrf \
    src/ros/src/ros_homebot_python/src/ros_homebot_python \
    src/ros/src/ros_homebot_teleop/nodes \
    src/ros/src/ros_homebot_teleop/src \
    src/overlay/src/ros_qr_tracker/nodes
