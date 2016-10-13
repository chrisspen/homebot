#!/usr/bin/env python
"""
Controls and publishes readings from the laser range finder.
"""
from __future__ import print_function

import os, time, sys
from math import pi
from threading import Thread
import yaml

import io
from PIL import Image as PilImage
import numpy
import cv2
from scipy.signal import medfilt2d
import rospy
import actionlib
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
from ros_homebot_lrf import constants as c
from ros_homebot_python.utils import fail, success

CONFIG_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../config')

# DEFAULT_LOG_LEVEL = rospy.DEBUG
DEFAULT_LOG_LEVEL = rospy.ERROR
# DEFAULT_LOG_LEVEL = rospy.FATAL

class CalibrationManager(object):
    """
    Walks the user through manually identify calibration markers.
    """
    
    def __init__(self, node, calibration_fn):
    
        self.node = node
    
        self.calibration_data = yaml.load(open(calibration_fn))
        print('calibration_data:', self.calibration_data)
        
        self.distances = self.calibration_data['distances'] # {pixel column: distance in mm}
        assert len(self.distances) >= 3, \
            'At least three or more calibration distances are needed. Only %i found in %s.' \
                % (len(self.distances), calibration_fn)
        
        self.pending_distances = sorted(self.distances.values())
        
        self.pixel_readings = None
        
        self.markers = {} # {pixel column: distance}
        
        self.current_distance = None
        self._last_distance = None 
    
    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.pixel_readings[x] == -1:
                print('No laser was detected in this column. Please select again.')
            else:
                self.markers[x] = self.current_distance
                self.current_distance = None
    
    def announce_next_marker(self):
        if self.current_distance:
            pass
        elif self.pending_distances:
            self.current_distance = self.pending_distances.pop(0)
            print('Please click on the center of the marker for distance %i.' % self.current_distance)
    
    def run(self):
        
        print('Collecting readings...')
        self.node.turn_laser_off()
        off_img = self.node.get_image_pil()
        self.node.turn_laser_on()
        on_img = self.node.get_image_pil()
        self.dist_readings, self.pixel_readings = self.node._lrf.get_distance(
            off_img,
            on_img,
            save_images_dir=os.path.expanduser('~'),#TODO
            as_pfc=True,
        )
        missing_count = sum(1 for _ in self.pixel_readings if _ < 0)
        missing_ratio = missing_count/float(len(self.pixel_readings))
        missing_percent = missing_ratio*100
        print('readings:', self.pixel_readings)
        print('missing_ratio:', missing_ratio)
        if missing_ratio > .2:
            print(('\nWarning! %.0f%% columns contain no laser readings. '
                'This may hinder manual classification. '
                'If you have problems locating markers, abort, remove any obstacles '
                'or bright lights that may interfere with measurements, and then re-run.'
                ) % missing_percent)
        
        print((
            '\nYou will now be asked to identify where %i distance markers are located. '
            'Press <enter> to begin.') % len(self.distances))
        raw_input()
        
        # Collect manual marker positions.
        img = self.node.get_image_cv2()
        height, width, channels = img.shape
        cv2.namedWindow('image')
        cv2.setMouseCallback('image', self.on_mouse)
        self.announce_next_marker()
        while self.pending_distances or self.current_distance is not None:
            self.announce_next_marker()
            cv2.imshow('image', img)
            k = cv2.waitKey(20) & 0xFF
            if k == 27:
                print('Escape key pressed. Aborting.')
                return
        cv2.destroyAllWindows()
        
        # Save updated calibration file.
        calibration_fn = os.path.join(CONFIG_DIR, 'calibration_%i.yaml' % width)
        write_to_file = False
        if raw_input('\nWrite calibration to %s? ' % calibration_fn).startswith('y'):
            fout = open(calibration_fn, 'w')
            write_to_file = True
        else:
            fout = sys.stdout
            
        print('readings:', self.pixel_readings, file=fout)
        print('distances:', file=fout)
        for k in sorted(self.markers):
            print('    %i: %i' % (k, self.markers[k]), file=fout)
        print('h: %s' % self.calibration_data['h'], file=fout)
        print('image_width: %i' % width, file=fout)
        print('image_height: %i' % height, file=fout)
        print('laser_position: %s' % self.calibration_data['laser_position'], file=fout)
        
        if write_to_file:
            print('\nCalibration file written to %s.' % calibration_fn)

class NoiseFilter(object):
    
    def __init__(self, name='medfilt', func_kwargs=None, history_size=3):
        from scipy import signal
        self.func = getattr(signal, name)
        self.func_kwargs = dict(kernel_size=3)
        
        self.history_size = history_size
        self.history_buckets = None
        
    def add(self, distances):
        
        # Initialize buckets.
        if self.history_buckets is None:
            self.history_buckets = [[] for _ in distances]
            
        for i, v in enumerate(distances):
            
            # Ignore missing values.
            if v < 0 and self.history_buckets[i]:
                continue
                
            # Add value to history.
            self.history_buckets[i].append(v)
            
            # Forget old values.
            if len(self.history_buckets[i]) > self.history_size:
                self.history_buckets[i].pop(0)
        
    def get(self):
        lst = [
            numpy.median(bucket)
            for bucket in self.history_buckets
        ]
        lst = self.func(lst, **self.func_kwargs)
        return lst

class LRF():
    
    def __init__(self):
        
        rospy.init_node('homebot_lrf', log_level=DEFAULT_LOG_LEVEL)
                
        # Cleanup when termniating the node
        rospy.on_shutdown(self.shutdown)
        
        # Overall loop rate: should be faster than fastest sensor rate
        self.rate = int(rospy.get_param("~rate", 50))
        r = rospy.Rate(self.rate)
        
        self._started = False
        self._state = c.OFF
        self._state_change_ts = None
        self._image_without_laser = None
        self._image_with_laser = None
        
        self.noise_filter = NoiseFilter()
        
        # The time when we started measuring distances.
        self._t0 = None
        
        ro = float(rospy.get_param('~ro', -0.0563705005565))
        rpc = float(rospy.get_param('~rpc', 0.00298408515511))
        h = float(rospy.get_param('~h', 22.5))
        laser_position = rospy.get_param('~laser_position', 'bottom')
        
        self.show_line_image = int(rospy.get_param('~show_line_image', 1))
        
        self.show_straightening = int(rospy.get_param("~straightening", 0))
        
        calibration_fn = rospy.get_param(
            "~calibration", os.path.join(CONFIG_DIR, 'calibration.yaml'))
        if not os.path.isfile(calibration_fn):
            tmp_fn = os.path.join(CONFIG_DIR, calibration_fn)
            if os.path.isfile(tmp_fn):
                calibration_fn = tmp_fn
        
        if self.show_straightening:
            # When testing the line laser level, use the non-calibrated LRF.
#             rpc, ro, h, laser_position = 0.00152454795238, -0.0529450746603, 22.5, 'bottom' # bad
#             rpc, ro, h, laser_position = 0.00298408515511, -0.0563705005565, 22.5, 'bottom' # good
            print('rpc, ro, h, laser_position:', rpc, ro, h, laser_position)
            self._lrf = LaserRangeFinder(
                ro=ro,
                rpc=rpc,
                h=h,
                laser_position=laser_position,
                track_progress_images=self.show_line_image,
                filter_outliers=int(rospy.get_param('~filter_outliers', 1)),
            )
            
        else:
            # Otherwise, load full calibration.
            if os.path.isfile(calibration_fn):
                print('Loading calibration file...')
                rpc, ro, h, laser_position = calibrate(calibration_fn)
                print('rpc, ro, h, laser_position:', rpc, ro, h, laser_position)
            
            self._lrf = LaserRangeFinder(
                ro=ro,
                rpc=rpc,
                h=h,
                laser_position=laser_position,
                track_progress_images=self.show_line_image,
                filter_outliers=int(rospy.get_param('~filter_outliers', 1)),
            )
        
        self.verbose = int(rospy.get_param("~verbose", 0))
        
        self.laser_pin = int(rospy.get_param("~laser_pin", c.LASER_PIN))
        
        self.distances_pub = rospy.Publisher('~scan', LaserScan, queue_size=1)
        
        self.line_image_pub = rospy.Publisher('~line/image', Image, queue_size=1)
        
        # This republishes images which don't have the laser line in them.
        self.image_off_pub = rospy.Publisher('~image/off', CompressedImage, queue_size=1)
        
        # This republishes images which do have the laser line in them.
        self.image_on_pub = rospy.Publisher('~image/on', CompressedImage, queue_size=1)
        
        self.line_columns_pub = rospy.Publisher(
            '~line/columns',
            msgs.LaserLineColumns,
            queue_size=1)
        
        self.rpi_gpio_set = rospy.ServiceProxy('/rpi_gpio/set_pin', DigitalWrite)
        
        self.camera_topic = rospy.get_param('~topic', '/raspicam/image')
        
        self.bridge = CvBridge()
        
        rospy.Service('~start', EmptySrv, self.start)
        rospy.Service('~stop', EmptySrv, self.stop)
        rospy.Service('~capture', EmptySrv, self.capture)
        
        self.show_marker = int(rospy.get_param("~marker", 0))
        markers = [] # [column]
        if self.show_marker:
            
            manager = CalibrationManager(self, calibration_fn)
            manager.run()
            
            return
        
        if int(rospy.get_param("~start", 0)):
            self.start()
        
        # Start polling the sensors and base controller
        while not rospy.is_shutdown():

            # Publish all sensor values on a single topic for convenience
            now = rospy.Time.now()

            #TODO:laser off, capture image, laser on, capture image, produce distance

            r.sleep()
    
    def log(self, *msg):
        if self.verbose:
            print(' '.join(map(str, msg)))
    
    def turn_laser_on(self):
        self.rpi_gpio_set(self.laser_pin, 1)
 
    def turn_laser_off(self):
        self.rpi_gpio_set(self.laser_pin, 0)
 
#     def on_gpio_pin_change(self, msg):
#         print('GPIO pin change:', msg
 
    def start(self, msg=None):
        """
        Launches a thread that infinitely publishes distance measurements.
        """
        if self._started:
            return
        self._started = True
        self._processing_thread = Thread(target=self.process)
        self._processing_thread.daemon = True
        self._processing_thread.start()
        return EmptyResponse()
        
    def stop(self, msg=None):
        """
        Stops the thread publishing distance measurements.
        """
        if not self._started:
            return
        self._started = False
        self._image_without_laser = None
        self._image_with_laser = None
        return EmptyResponse()
  
    def capture(self, msg=None):
        """
        Essentially a blocking version of start(), except only runs one iteration, then exits.
        """
        self._started = True
        self.process(iterations=1)
  
    def normalize_image_cv2(self, msg):
        if isinstance(msg, CompressedImage):
            pil_image = self.normalize_compressed_image(msg)
            cv_image = numpy.array(pil_image)
            return cv_image
        else:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgra8")
            return cv2.cvtColor(cv_image, cv2.COLOR_BGRA2RGB)
         
    def normalize_image_pil(self, msg):
        if isinstance(msg, CompressedImage):
            return self.normalize_compressed_image(msg)
        else:
            cv_image = self.normalize_image_cv2(msg)
            return PilImage.fromarray(cv_image)
    
    def normalize_compressed_image(self, msg):
        return PilImage.open(io.BytesIO(bytearray(msg.data)))
        
    def get_image(self):
        if 'compressed' in self.camera_topic:
#             print('waiting for compressed image from %s' % self.camera_topic)
            return rospy.wait_for_message(self.camera_topic, CompressedImage)
        else:
#             print('waiting for raw image from %s' % self.camera_topic)
            return rospy.wait_for_message(self.camera_topic, Image)
        
    def get_image_pil(self):
        return self.normalize_image_pil(self.get_image())
        
    def get_image_cv2(self):
        return self.normalize_image_cv2(self.get_image())
 
    def process(self, iterations=0):
        """
        Continually captures distance measurements and publishes the data
        via standard ROS messages.
        
        Parameters
        ----------
        iterations : int
            If positive, description the number of measurement loops to perform before exiting.
            Otherwise, an infinite loop will be performed. 
        """
        
        max_straight_readings = 10
        straight_readings = []
        
        self.log('Processing thread started.')
        count = 0
        while self._started:
            count += 1
            t00 = time.time()
            
            # Ensure laser starts off.
            t0 = time.time()
            self.turn_laser_off()
            self.log('laser off:', time.time() - t0)
            
            # Save camera image.
            t0 = time.time()
            off_img = self.get_image_pil()
            #off_img.save(os.path.expanduser('~/off_img.jpeg'))#TODO
            image_message = self.bridge.cv2_to_compressed_imgmsg(numpy.array(off_img), dst_format='jpg')
            self.image_off_pub.publish(image_message)
            self.log('image capture:', time.time() - t0)
            
            # Ensure laser is on.
            t0 = time.time()
            self.turn_laser_on()
            self.log('laser on:', time.time() - t0)
            
            # Save camera image.
            on_img = self.get_image_pil()
#             on_img.save(os.path.expanduser('~/on_img.jpeg'))#TODO
            image_message = self.bridge.cv2_to_compressed_imgmsg(numpy.array(on_img), dst_format='jpg')
            self.image_on_pub.publish(image_message)
            self.log('image capture:', time.time() - t0)

            # Turn laser off again while we process.
            self.turn_laser_off()
            
            # Calculate distance
            t0 = time.time()
            distances, pfc = self._lrf.get_distance(
                off_img,#self._image_without_laser,
                on_img,#self._image_with_laser,
                save_images_dir=os.path.expanduser('~'),#TODO
                as_pfc=self.show_straightening,
            )
            assert len(distances) == len(pfc)
            self.log('dist calc:', time.time() - t0)
            
            # Filter distances to remove noise.
            self.noise_filter.add(distances)
            distances = self.noise_filter.get()
            
            if self.show_straightening:
                level_variance = numpy.var([_ for _ in distances if _ >= 0])
                print('raw pixels:', ' '.join(map(lambda v: str(int(v)), compress_list(distances))), 'level variance (should be close to 0):', level_variance)
                sys.stdout.flush()
                straight_readings.append(level_variance)
                if len(straight_readings) >= max_straight_readings:
                    avg_level_offset = sum(straight_readings)/float(len(straight_readings))
                    is_good = avg_level_offset <= c.MAX_LEVEL_VARIANCE
                    color_func = success if is_good else fail
                    print('average level variance:', color_func('%.02f' % avg_level_offset))
                    if is_good:
                        print('Laser is level. Good job!')
                    else:
                        print('This is not good. Ensure your line laser is level and that it is projecting against a flat wall about 50 cm away.')
                    rospy.signal_shutdown('complete')
                    return
#             else:
# #                 print('distances0:', distances)
# #                 print('distances2:', compress_list(distances))
#                 pass
#                 break

            # Publish line image.
            #http://stackoverflow.com/a/14140796/247542
            if self.show_line_image:
                t0 = time.time()
                pil_image = self._lrf.out3.convert('RGB')
                cv_image = numpy.array(pil_image)
                #cv2.cvtColor(cv_image, cv2.COLOR_BGRA2RGB)
#                 cv_image = cv_image[:, :, ::-1].copy()
                image_message = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')#outputs pipe?
                self.line_image_pub.publish(image_message)
                self.log('line pub:', time.time() - t0)
                
#             
#             #http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
            #http://answers.ros.org/question/198843/need-explanation-on-sensor_msgslaserscanmsg/
            width, height = on_img.size
            detection_angle = self._lrf.horz_fov_deg
            
            line_msg = msgs.LaserLineColumns()
            line_msg.width = width
            line_msg.height = height
            line_msg.line = pfc
            self.line_columns_pub.publish(line_msg)
            
            msg = LaserScan()
            msg.angle_max = (detection_angle/2.)*pi/180.
            msg.angle_min = -msg.angle_max
            msg.angle_increment = detection_angle*pi/180./width
            #msg.time_increment = 0#time.time() - self._t0
            #msg.scan_time = None
            msg.range_min = -1 # 0mm
            msg.range_max = 10000 # 10meters
            msg.ranges = [_*1000 for _ in distances]
#             msg.intensities = None
            self.distances_pub.publish(msg)

            tdd = time.time() - t00
            self.log('full tdd:', tdd)
            
            if iterations > 0 and count >= iterations:
                break

        self._started = False
        self.log('Processing thread stopped.')
 
    def shutdown(self):
        rospy.loginfo("Shutting down the node...")
        self.turn_laser_off()
        
if __name__ == '__main__':
    LRF()
