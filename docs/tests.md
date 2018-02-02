Tests
=====

These are tests for evaluating milestones and basic functionality. They are listed roughly in order of dependence. A test at the end usually requires passing one of the earlier tests.

Collision avoidance
-------------------

Moves the platform forward at 25% speed until an obstacle is detected via ultrasonic, at which point it should immediately halt before colliding.

    rosrun ros_homebot motion.py
    rosrun ros_homebot forward_client.py

Edge avoidance
--------------

Move the platofrm foward at 25% speed until an edge or cliff is detected, at which point it should immediately halt before falling.

    rosrun ros_homebot motion.py
    rosrun ros_homebot forward_client.py

QR tracking
-----------

Search for and track a QR code using the head camera. Head should pan and tilt to track code as it moves.

    rosrun ros_homebot track_qr.py

QR following
------------

Search for and track a QR code, maintaining a fixed distance from the code at all times. This is meant to be tested on a treadmill, where the QR code is mounted at the front of the treadmill facing the robot, with the speed slowly being increased from 0mph to 1-2mph. The robot should be able to proportionally increase it's speed and adjust drift to keep itself centered in the middle of the treadmill width-wise and at a relatively fixed distance roughly in the middle length-wise. 

    rosrun ros_homebot treadmill.py

Auto docking
------------

The docking station, identified with a unique QR code, is placed within eyesight of the robot. The robot should then identify the docking station, and manuver itself into docking position so that it begins recharging.

    todo

SLAM
----

Map and localize itself within one or more rooms. After mapping is complete, a room will be specified on a map, and the robot should be able to autonomously navigate to that room.

    todo

Kidnapped
---------

After mapping a small group of rooms, the robot will be deactivated, manually relocated to a different room, and then reactivated, at which point it will have to navigate to the original room it was in.

    todo

Baby duck
---------

Identify and track a target as they move around.

    todo

Hide and go seek
----------------

Identify target's face and body pattern, and then deactivate camera and count to 10 while target moves to a different location. At the end of the countdown, robot will then begin searching for the person, stopped when the person is identified. The rooms should be mapped as it travels so areas are not redundently searched unless necessary.

    todo
