Controls a simple home-made laser range finder composed of a camera and line laser.

Input:

    video stream
    
Output:

    laser state via a GPIO pin
    laser range finder stream

Dependencies
------------

First ensure the camera capture is running with:

    roslaunch ros_homebot raspicam_compressed_320.launch
    export ROS_MASTER_URI=http://{robot_hostname}.local:11311
    rosrun image_view image_view image:=/camera/image _image_transport:=compressed

As well as the RPi_GPIO node:

    roslaunch ros_homebot rpi_gpio.launch

Calibration
-----------

Start by leveling the line laser. Point the laser perpendicular to a large flat surface, such as a wall, and run:

    rosrun ros_homebot_lrf lrf_node.py _straightening:=1 _verbose:=1 _start:=1
    
and adjust the angle of the line laser until `level_offset` is as close to 0 as possible. A value within +/- 10 is usually sufficient.

You may want to put a dab of superglue on the rim of the laser, to help ensure it stays correctly positioned. If it loosens and rotates, this will lessen the accuracy of distance measurements.

Next, place some simple objects at varying distances in front of the laser and run:

    rosrun ros_homebot_lrf lrf_node.py _marker:=1
    
This will display a window showing an image of the line laser.

Going from left to right, manually measure the distance to where the laser touches each object, and click on those spots in the image.

When enough spots have been selected, the window will close and a calibration file will be written to `config/calibrate.yaml`. Open this file and add the manual distance measurements to the `readings` dictionary, along with the `h` value.

Output
------

You can see a processed version of the image showing the line estimate by running:

    rosrun image_view image_view image:=/homebot_lrf/line/image

References
----------

http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
