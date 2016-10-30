#!/bin/bash
# Initializes a shell, not running on the robot, for ROS.

. ./setup.bash

_CURRENT_DIR=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)
_ROS_MASTER_URI=http://localhost:11311

if [ -f "$_CURRENT_DIR/setup_remote_local.bash" ]; then
    . $_CURRENT_DIR/setup_remote_local.bash
fi
export ROS_MASTER_URI=$_ROS_MASTER_URI;
