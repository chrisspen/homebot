#!/bin/bash
# Call by sourcing, e.g. source setup.bash
#http://wiki.ros.org/catkin/Tutorials/workspace_overlaying

_CURRENT_DIR=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)
_ROS_VERSION=kinetic

if [ -f "$_CURRENT_DIR/setup_local.bash" ]; then
    . $_CURRENT_DIR/setup_local.bash
fi

source /opt/ros/$_ROS_VERSION/setup.bash
source $_CURRENT_DIR/../../.env/bin/activate
source $_CURRENT_DIR/../overlay/devel/setup.bash --extend
source $_CURRENT_DIR/devel/setup.sh --extend
