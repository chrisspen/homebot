Laser Range Finder Calibration
==============================

Ensure the Raspberry Pi camera module is first calibrated.

Leveling
--------

To begin the laser range finder (LRF) calibration, begin by ensuring the line laser is level relative to the camera. Since distance is calculated based on the laser's vertical position in the image, it's important that the line laser is equally level throughout the image. A laser reading close to the center of the image will be interpreted as an object far away, whereas a reading close to the bottom of the imaeg will be interpreted as an object close by. If the laser is tilted relative to the camera, then a flat wall perfectly perpendicular to the camera may be interpreted nonsensically as being both very close and almost infinitely far away.

Face the platform at a flat wall, about 50 cm away, with no obstructions. Then run on the Pi:

    roslaunch ros_homebot raspicam_compressed_320.launch
    roslaunch ros_homebot rpi_gpio.launch

and from your deployment machine, run:

    roslaunch ros_homebot lrf_straightener.launch

This will sample the laser readings and calculate the variance in the vertical position of the laser in the camera image. If the variance is 0, the laser is perfectly aligned. The variance should be no more than 10. If the reading is more than this, assure you're oriented the platform properly (if the wall is not flat or there are obstructions, false readings will occur) and that the line laser is level. Gently twist the line laser and re-run the process until a low variance is reported.

Calibration
-----------

Next we'll associate the raw line laser readings with real-world distance measurements.

Begin by setting up 6 distance markers evenly around the platform within the 90 degree view of the camera. Use strings of known lengths ranging from:

* 5cm
* 10cm
* 50cm
* 100cm
* 200cm
* 300cm

Activate the line laser with:

    rosservice call /rpi_gpio/set_pin 20 1

Confirm the line laser shines equally across all marker labels, then deactivate the line laser with:

    rosservice call /rpi_gpio/set_pin 20 0

Then run:

    rosrun ros_homebot_lrf lrf_node.py _marker:=1

This will display a window showing an image of the line laser. Follow the on-screen instructions.

Finally, confirm calibration has been set correctly but running a continuous laser ranger finder with:

    roslaunch ros_homebot lrf_compressed_320_debug.launch

and visualizing it with:

    rosrun ros_homebot_lrf lrf_plot.py

Slowly move your hand in front of the laser and confirm the visualization changes accordingly.
