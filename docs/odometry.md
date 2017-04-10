Odometry
========

Odometry measures the robot's translation and rotation from an arbitrary reference point. This is information necessary for planning robot movements and doing higher level actions like mapping and path planning.

Testing
-------

To confirm the robot is generating correct [odometry messages](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html), first start nodes to control the torso and visualize the robot:

    roslaunch ros_homebot_base serial_torso.launch
    roslaunch ros_homebot_description display_real.launch

Then command the motors to move foward are quarter speed using the differential format:

    rostopic pub --once /torso_arduino/motor/speed std_msgs/Int16MultiArray "{layout:{dim:[{label: '', size: 0, stride: 0}], data_offset: 0}, data:[64, 64]}"

or Twist format, which units are in meters:

    rostopic pub -1 /torso_arduino/cmd_vel geometry_msgs/Twist -- '[0.0875, 0.0, 0.0]' '[0.0, 0.0, 0.0]'

Note about speeds that the Arduino represents speed internally as an integer value between -255 to +255. A negative value is reverse. A positive value is forward. A zero value is stationary. To set speed via a Twist message, which only uses meters, the meter value has to be converted to the internal integer value. This is done in Motioncontroller.h via the `VELOCITY_TO_SPEED` ratio, which represents a conversion from a literal meter/second velocity value to a relative integer velocity value.

This value is about 728.57, and was determined by empirical measurement. However, it can vary based on nuances in the motor, motor controller, and tread friction with the ground.

To convert meters to integer value, you'd calculate `VELOCITY_TO_SPEED*METERS = SPEED`.

To convert an integer value to meters, you'd calculate `METERS = SPEED/VELOCITY_TO_SPEED`.

Since the maximum relative speed is 255, that means the maximum absolute speed is `255/728.57` = 0.35 meters/second.

Now look at the Rviz screen. If your motors are running and odometry messages are being generated correctly, you should see the robot moving forward in Rviz.

If you don't see anything, you'll need to diagnose the problem. Start by confirming odometry messages are being sent with:

    rostopic echo /torso_arduino/odom

If the robot is correctly shown to be moving, stop the robot with:

    rostopic pub --once /torso_arduino/halt std_msgs/Empty
