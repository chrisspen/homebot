#!/usr/bin/env python
from __future__ import print_function

import os, time, sys
from math import pi, ceil
from threading import Thread, RLock
import yaml

import io
from PIL import Image as PilImage
import numpy
import cv2
import rospy
from sensor_msgs.msg import CompressedImage, Image, LaserScan
from std_srvs.srv import Empty as EmptySrv, EmptyResponse
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

from rpi_gpio.srv import DigitalWrite
from rpi_gpio import msg as msgs

from ros_homebot_msgs.srv import *
from ros_homebot_msgs import msg as msgs
from ros_homebot_lrf.lrf import LaserRangeFinder, calibrate
from ros_homebot_lrf.utils import compress_list
from ros_homebot_python.utils import fail, success

# DEFAULT_LOG_LEVEL = rospy.DEBUG
DEFAULT_LOG_LEVEL = rospy.ERROR
# DEFAULT_LOG_LEVEL = rospy.FATAL

# Colors (B, G, R)
WHITE = (255, 255, 255)
GRAY = (200, 200, 200)
BLACK = (0, 0, 0)
RED = (0, 0, 255)

def create_blank(width, height, color=(0, 0, 0)):
    """Create new image(numpy array) filled with certain color in BGR"""
    image = np.zeros((height, width, 3), np.uint8)
    # Fill image with color
    image[:] = color
    return image

def draw_half_circle_rounded(image, degrees, thickness=-1, radius=100, angle=0, color=BLACK):
    """
    degrees: starts from the middle right-hand side and goes counter-clockwise
    angle: rotates shape counter-clockwise
    thickness: -1=fills in shape, > 0 = sets the line
    """
    height, width = image.shape[0:2]
    
    # Ellipse parameters
    #center = (width / 2, height /2)
    center = (width / 2, height)
    axes = (radius, radius)

    angle = float(angle)
    startAngle = 0.0
    endAngle = float(degrees)

    # http://docs.opencv.org/modules/core/doc/drawing_functions.html#ellipse
    cv2.ellipse(image, center, axes, angle, startAngle, endAngle, color, thickness, cv2.CV_AA)


class LRFPlotter():
    
    def __init__(self):
        
        rospy.init_node('homebot_lrf_plotter', log_level=DEFAULT_LOG_LEVEL)
        
        self.lock = RLock()
        
        self.width = int(rospy.get_param("~width", 800))
        
        self.height = int(rospy.get_param("~height", 600))
        
        self.topic = rospy.get_param("~topic", '/homebot_lrf/scan')
        
        # Cleanup when termniating the node
        rospy.on_shutdown(self.shutdown)
        
        rospy.Subscriber(self.topic, LaserScan, self.process_laserscan)
        
        self.distances = None
         
        self.name = 'Laser Range Finder Measurements'
        print('Press escape to exit.')
        cv2.namedWindow(self.name)
        while not rospy.is_shutdown():
            image = self.update_image()
            if image is None:
                time.sleep(0.1)
                continue
            cv2.imshow(self.name, image)
            k = cv2.waitKey(20) & 0xFF
            if k == 27:
                print('Escape key pressed. Exiting.')
                return
        cv2.destroyAllWindows()
    
    def update_image(self):
        with self.lock:
            if not self.distances:
                return

        # Create new blank 300x150 white image
        pixels_to_mm = self.width/300.0
        width, height = self.width, self.height
        image = create_blank(width, height, color=WHITE)
        max_degrees = 90
        degrees_per_reading = float(max_degrees)/len(self.distances)
#         print('degrees_per_reading:', degrees_per_reading)
        prior_angles = 0
        for i, distance in enumerate(self.distances):
#             print('distance:', distance)
            if distance <= 0:
                pixel_distance = height
                fill_color = GRAY
                line_color = None
            else:
                #pixel_distance = int(ceil(pixels_to_mm * distance))
                pixel_distance = int(round(pixels_to_mm * distance))
                fill_color = BLACK
                line_color = RED
#             print('pixel_distance:', pixel_distance)
             
            angle = -90-45 + prior_angles
             
            # Fill in.
            draw_half_circle_rounded(
                image,
                degrees=degrees_per_reading,
                angle=angle,
                radius=pixel_distance,
                color=fill_color,
                thickness=-1
            )
             
            # Draw border.
            if line_color:
                draw_half_circle_rounded(
                    image,
                    degrees=degrees_per_reading,
                    angle=angle,
                    radius=pixel_distance,
                    color=line_color,
                    thickness=3,
                )
             
            prior_angles += degrees_per_reading
            
        return image

    def process_laserscan(self, msg):
#         print('process_laserscan.distances:', len(msg.ranges))
        with self.lock:
            # Convert meters to mm.
            self.distances = [_/1000. for _ in msg.ranges]
 
    def shutdown(self):
        rospy.loginfo("Shutting down the node...")
        
if __name__ == '__main__':
    LRFPlotter()
