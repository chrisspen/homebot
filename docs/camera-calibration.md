Camera Calibration
==================

Calibrating the Raspberry Pi camera module
------------------------------------------

To calibrate the Raspberry Pi camera module, start by running the camera node on the Pi with:

    roslaunch ros_homebot raspicam_raw.launch

Then, on your deployment machine, initialize your shell with:

    . src/ros/setup.bash
    export ROS_MASTER_URI=http://rae.local:11311
    
Ensure you deployment machine can access the camera stream by running:

    rosrun image_view image_view image:=/camera/image

You should see a small window showing the camera stream with very high (3-6 second) latency.

Close this window by focussing the terminal and pressing ctrl+c.

Then run the [ROS monocular calibration script](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration) with:

    rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.025 image:=/camera/image camera:=/camera --no-service-check --camera_name=raspicam

Move the checkerboard pattern around in front of the camera, going slowly to ensure the calibration script can identify the pattern, indicated by showing an overlay of colored lines between the checkerboard vertices. Slowly move the checkerboard left, right, up and down, tilting it forwards and backwards, close to the camera and then farther away from the camera. When enough data has been collected, the lines on the right hand side of the screen should turn blue and the "Calibrate" button should highlight. Click it once, then click "Save", and then exit the window.

Note, the "Commit" button attempts to send the calibration parameters directly to the camera node via a service call. However, this is not supported, so it will only throw an error in the console.

The calibration data will be printed to the console. It will also be saved to a file at /tmp/calibrationdata.tar.gz.

Extract the file `ost.yaml` from calibrationdata.tar.gz and copy it to raspicam_node/calibrations/camera.yaml.

Push the new file to the Pi and test the new calibration by running on the Pi:

    roslaunch ros_homebot raspicam_compressed_320.launch

Confirm the compressed video feed looks correct by viewing it from your deployment machine with:

    rosrun image_view image_view image:=/camera/image _image_transport:=compressed
