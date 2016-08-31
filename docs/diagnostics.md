Diagnostics
===========

A diagnostic program is included to confirm all systems are running correctly.

This program will interfere with any production use of the robot, so before running it, shutdown any high-level control node or actions, and place the robot on its maintenance stand.

The program is designed to be run from the deployment machine. Run it with:

    rosrun ros_homebot diagnostic.py

This will walk you through verifying all systems. Most of the tests will require some manual interaction.

You can isolate just the head or torso section be tested by specifying the section parameter like:

    rosrun ros_homebot diagnostic.py _section:=head

You can isolate a specific part by specifying that part's name via the part parameter like:

    rosrun ros_homebot diagnostic.py _section:=head _part:=pan_change_sensor
