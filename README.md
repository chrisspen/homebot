Homebot
============================================================

<img src="docs/img/bot/banner.png?raw=true" />

Homebot is a low cost open source robot designed for research and experimentation with [ROS](http://www.ros.org/).

Basic features include:

* head that can rotate 360 degrees horizontally and 180 degrees vertically
* monocular camera fitting with a laser ranger finder
* IMU with magnetic compass for navigation
* ultrasonic range finders and IR distance sensor for forward obstacle and cliff detection
* Raspberry Pi 3 for running ROS and high-level control

[Click here for documentation](http://chrisspen.github.io/homebot).

Shortcuts:

To deploy all code changes to the production robot, run:

    fab prod deploy.push
