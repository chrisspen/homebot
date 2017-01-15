#! /usr/bin/env python
"""
Utility for running a level 2 diagnostic.
"""
from __future__ import print_function

import os
import time
import sys
import operator
from threading import RLock
from functools import partial
import multiprocessing

# import roslib
#roslib.load_manifest('ros_homebot')
import rospy
import rosnode

import ros_homebot_msgs.srv
import ros_homebot_msgs.msg
from ros_homebot_python import constants as c
from ros_homebot_python.node import (
    subscribe_to_topic,
    get_service_proxy,
    packet_to_message_type,
    set_line_laser,
    say,
)
from ros_homebot_python import utils
from ros_homebot_python.robot import Robot
from ros_homebot_python.exceptions import DeviceNotFound
from ros_homebot_lrf.utils import compress_list, list_to_str as line_list_to_str

YN = 'yn'
Y = 'y'
N = 'n'
ENTER = '<enter>'

def process_is_alive(pid):        
    """ Check For the existence of a unix pid. """
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    else:
        return True

class MessageHandler(object):
    """
    Tracks ROS messages and allows us to wait for specific conditions.
    """
    
    def __init__(self):
        self._lock = RLock()
        self._last_msg = None
        self._condition = None

    def wait_for_condition(self, condition, **kwargs):
        """
        condition := may be:
        
            1. dictionary of name/value pairs to compare against the message
            2. a callable that accepts the message and returns True if the condition has been met
            
        quiet := if True, only displays message once
        """
        
        timeout = kwargs.pop('timeout', 10)
        message = kwargs.pop('message', None)
        message_update = kwargs.pop('message_update', False)
        quiet = kwargs.pop('quiet', False)
        
        t0 = time.time()
        self._condition = condition
        message_shown = False
        while time.time() - t0 < timeout and not rospy.is_shutdown():
            with self._lock:
            
                if message:
                    
                    if message_update:
                        sys.stdout.write('\r%i ' % (timeout - (time.time()-t0)))
                        
                    _message = message
                    if self._last_msg:
                        msg_dict = dict(
                            (_name, getattr(self._last_msg, _name))
                            for _name in self._last_msg.__slots__
                        )
                        _message = _message.format(**msg_dict)
                        
                    if not quiet or (quiet and not message_shown):
                        sys.stdout.write(_message)
                        if quiet:
                            sys.stdout.write('\n')
                        sys.stdout.flush()
                        message_shown = True
                    
                if self._last_msg:
                    match = self.matches(condition, self._last_msg)
                    if match:
                        return True
                        
            time.sleep(.1)
        return False

    def matches(self, condition, other):
        if callable(condition):
            return condition(other)
        else:
            match = True
            for k, v in condition.items():
                
                # Extra operator from name.
                k_parts = k.split('__')
                if len(k_parts) >= 2:
                    k = k_parts[0]
                    op = {
                        'lte': operator.le,
                        'gte': operator.ge,
                        'lt': operator.lt,
                        'gt': operator.gt,
                    }[k_parts[1]]
                else:
                    op = operator.eq
                
                if not op(getattr(other, k), v):
                    match = False
                    break
            return match

    def __call__(self, msg):
        """
        Called when a message for this type is received.
        """
        with self._lock:
            if self._condition:
                match = self.matches(self._condition, msg)
                if match:
                    self._last_msg = msg
            else:
                self._last_msg = msg

class Check(object):
    """
    Represents a specific check to be done, usually with manually actuation.
    """
    
    def __init__(self, parent, **kwargs):
        self.parent = parent
        
        self.pre_message = kwargs.pop('pre_message')
        self.msg_type = kwargs.pop('msg_type', None)
        self.msg_condition = kwargs.pop('msg_condition', None)
        self.msg_timeout = kwargs.pop('msg_timeout', 20)
        self.msg_update = kwargs.pop('msg_update', True)
        self.msg_quiet = kwargs.pop('msg_quiet', False)
        self.answer_options = kwargs.pop('answer_options', None)
        self.success_answer = kwargs.pop('success_answer', None)
        self.pre_callback = kwargs.pop('pre_callback', None)
        self.post_callback = kwargs.pop('post_callback', None)
        
        self.success = None

    def run(self):
        
        ret = self.pre_callback and self.pre_callback()
            
        if self.msg_type:
            # Wait for a packet in response to a user action.
            ret = self.parent.handlers[self.msg_type].wait_for_condition(
                condition=self.msg_condition,
                timeout=self.msg_timeout,
                message=self.pre_message + ' ',
                message_update=self.msg_update,
                quiet=self.msg_quiet,
            )
            self.success = ret
        else:
            # Prompt the user for direct feedback.
            while 1:
                ao = ''
                if self.answer_options:
                    ao = ' [%s] ' % self.answer_options
                
                ret = raw_input(self.pre_message + ao).strip()
                if not ret and self.answer_options and self.answer_options != ENTER:
                    ret = self.answer_options[0]
                    
                if self.answer_options == ENTER and not ret:
                    self.success = True
                    break
                elif self.answer_options and ret not in self.answer_options:
                    print('Invalid response.')
                else:
                    self.success = self.success_answer == ret
                    break
                    
        if self.success:
            print(utils.success('Check passed!'))
        else:
            print(utils.fail('Check failed!'))
            
        ret = self.post_callback and self.post_callback()

class Diagnostic(Robot):
    """
    Manual diagnostic tool for checking base functionality.
    
    Remember, you can see published torso topics with:
    
        rostopic list | grep -i torso
        
    """
    
    raspicam_process = None
    
    def __init__(self, name):
        
        self.running = True
        
        rospy.on_shutdown(self.shutdown)

        print('Looking for section...')
        self.section = None
        try:
            self.section = (rospy.get_param('~section') or '').upper() # head|torso
            assert self.section in (c.HEAD, c.TORSO, '')
            if self.section == 'none':
                self.section = ''
        except KeyError:
            pass
        if self.section:
            print('Checking section %s.' % self.section)
        else:
            print('No section specified. Checking all sections.')
        
        print('Looking for part...')
        self.part = None
        try:
            self.part = (rospy.get_param('~part') or '').lower().strip()
            if self.part == 'none':
                self.part = ''
        except KeyError:
            pass
        if self.part:
            print('Checking part %s.' % self.part)
        else:
            print('No part specified. Checking all part.')
        
        skip_device = int(rospy.get_param('~skip_device_check', 0))
        
        print('skip_device:', skip_device)
        
        if not skip_device:
            if not self.section or self.section == c.HEAD:
                while not rospy.is_shutdown() and self.running:
                    try:
                        print('Looking for head...')
                        device = utils.find_serial_device(c.HEAD)
                        break
                    except DeviceNotFound:
                        self.wait_until_continue('Attach the head via USB.')
                        
            if not self.section or self.section == c.TORSO:
                while not rospy.is_shutdown() and self.running:
                    try:
                        print('Looking for torso...')
                        device = utils.find_serial_device(c.TORSO)
                        break
                    except DeviceNotFound:
                        self.wait_until_continue('Attach the torso via USB.')
        
        if not self.section or self.section == c.HEAD:
            print('Pinging head node...')
            assert rosnode.rosnode_ping('/head_arduino', max_count=1, verbose=True), \
                'Head arduino node not detected.'
            
        if not self.section or self.section == c.TORSO:
            print('Pinging torso node...')
            assert rosnode.rosnode_ping('/torso_arduino', max_count=1, verbose=True), \
                'Torso arduino node not detected.'
        
        if (not self.section or self.section == c.HEAD) and (not self.part or self.part == 'lrf'):
            print('Pinging LRF node...')
            assert rosnode.rosnode_ping('homebot_lrf', max_count=1, verbose=True), \
                'LRF node not detected.'
            print('Pinging Raspicam node...')
            assert rosnode.rosnode_ping('raspicam_node', max_count=1, verbose=True), \
                'Raspicam node not detected.'
            
        self.check_results = {} # {name: success}
        
        self.checks = []
        
        self.handlers = {} # {msg: latch}
        
        # Torso arduino subscriptions.
        self.subscribe_torso(c.ID_BUMPER)
        self.subscribe_torso(c.ID_EDGE)
        self.subscribe_torso(c.ID_ULTRASONIC)
        self.subscribe_torso(c.ID_STATUS_BUTTON)
        self.subscribe_torso(c.ID_IMU_ACCELEROMETER)
        self.subscribe_torso(c.ID_IMU_EULER)
        self.subscribe_torso(c.ID_IMU_MAGNETOMETER)
        self.subscribe_torso(c.ID_ARDUINO_TEMP)
        self.subscribe_torso(c.ID_BATTERY_VOLTAGE)
        self.subscribe_torso(c.ID_EXTERNAL_POWER)
        
        # Head arduino subscriptions.
        self.subscribe_head(c.ID_PAN_CENTERMARK)
        self.subscribe_head(c.ID_PAN_ANGLE)
        
        # Other head subscriptions.
        self.handlers[ros_homebot_msgs.msg.LaserLineColumns] = MessageHandler()
        rospy.Subscriber(
            '/homebot_lrf/line/columns',
            ros_homebot_msgs.msg.LaserLineColumns,
            self.record_topic)
        
        print('Subscribed to topics.')
        
        self.add_checks()
        
        self.wait_until_continue(
            'Place the torso on struts so the treads are at least 3cm off the ground.')
        
        # Run all checks.
        total = len(self.checks)
        passed = 0
        for i, check in enumerate(self.checks):
            sys.stdout.write('%02i/%02i ' % (i+1, total))
            check.run()
            passed += check.success
            if rospy.is_shutdown():
                sys.exit()
            
        print('-'*80)
        print('Passed %i of %i checks.' % (passed, total))
        score = passed/float(total)*100
        print('Score: %.02f%%' % score)
    
    def add_checks(self):
        
        if not self.section or self.section == c.HEAD:
            self.add_head_checks()
        
        if not self.section or self.section == c.TORSO:
            self.add_torso_checks()
    
    def add_head_checks(self):
        
        # Ultrabright LEDs.
        if not self.part or self.part == 'ultrabright':
            self.checks.append(Check(
                self,
                pre_message='Are the ultrabright LEDs off?',
                answer_options=YN,
                success_answer=Y,
                pre_callback=partial(self.set_ultrabright, value=0),
            ))
            self.checks.append(Check(
                self,
                pre_message='Are the ultrabright LEDs on?',
                answer_options=YN,
                success_answer=Y,
                pre_callback=partial(self.set_ultrabright, value=255),
            ))
            self.checks.append(Check(
                self,
                pre_message='Are the ultrabright LEDs off?',
                answer_options=YN,
                success_answer=Y,
                pre_callback=partial(self.set_ultrabright, value=0),
            ))
            
        # RGB LED.
        if not self.part or self.part == 'rgbled':
            self.checks.append(Check(
                self,
                pre_message='Is the RGB LED off?',
                answer_options=YN,
                success_answer=Y,
                pre_callback=partial(self.set_rgbled, value=(0, 0, 0)),
            ))
            self.checks.append(Check(
                self,
                pre_message='Is the RGB LED red?',
                answer_options=YN,
                success_answer=Y,
                pre_callback=partial(self.set_rgbled, value=(254, 0, 0)),
            ))
            self.checks.append(Check(
                self,
                pre_message='Is the RGB LED green?',
                answer_options=YN,
                success_answer=Y,
                pre_callback=partial(self.set_rgbled, value=(0, 254, 0)),
            ))
            self.checks.append(Check(
                self,
                pre_message='Is the RGB LED blue?',
                answer_options=YN,
                success_answer=Y,
                pre_callback=partial(self.set_rgbled, value=(0, 0, 254)),
            ))
            self.checks.append(Check(
                self,
                pre_message='Is the RGB LED off?',
                answer_options=YN,
                success_answer=Y,
                pre_callback=partial(self.set_rgbled, value=(0, 0, 0)),
            ))
        
        # Laser
        if not self.part or self.part == 'laser':
            self.checks.append(Check(
                self,
                pre_message='Is the laser off?',
                answer_options=YN,
                success_answer=Y,
                pre_callback=partial(set_line_laser, 0),
            ))
            self.checks.append(Check(
                self,
                pre_message='Is the laser on?',
                answer_options=YN,
                success_answer=Y,
                pre_callback=partial(set_line_laser, 1),
            ))
            self.checks.append(Check(
                self,
                pre_message='Is the laser off?',
                answer_options=YN,
                success_answer=Y,
                pre_callback=partial(set_line_laser, 0),
            ))
        
        # Sound.
        if not self.part or self.part == 'speaker':
            say_text = 'Testing one two three. Testing one two three.'
            self.checks.append(Check(
                self,
                pre_message='Did you hear a voice saying "%s"?' % say_text,
                answer_options=YN,
                success_answer=Y,
                pre_callback=partial(say, say_text),
            ))
            
        # Camera
        if not self.part or self.part == 'raspicam':
            self.checks.append(Check(
                self,
                pre_message='Do you see a camera window with an active video feed?',
                answer_options=YN,
                success_answer=Y,
                pre_callback=self.show_camera_window,
                post_callback=self.hide_camera_window,
            ))
        
        # Laser range finder (laser + camera)
        #TODO
#         if not self.part or self.part == 'lrf':
#             
#             self.checks.append(Check(
#                 self,
#                 pre_message='Clear the area directly infront of the camera to over 10cm.',
#                 msg_type=ros_homebot_msgs.msg.LaserLineColumns,
#                 msg_condition=self.is_line_laser_clear,
#                 msg_update=True,
#                 msg_quiet=True,
#             ))
#             
#             self.checks.append(Check(
#                 self,
#                 pre_message='Obstruct the area directly infront of the camera to about 10cm.',
#                 msg_type=ros_homebot_msgs.msg.LaserLineColumns,
#                 msg_condition=self.is_line_laser_obstructed,
#                 msg_update=True,
#                 msg_quiet=True,
#             ))
        
        # Pan centering.
        if not self.part or self.part == 'pan_centermark':
            self.checks.append(Check(
                self,
                pre_message='Manually center the head.',
                msg_type=ros_homebot_msgs.msg.PanCentermarkChange,
                msg_condition={'state': 1},
            ))
            self.checks.append(Check(
                self,
                pre_message='Manually  un-center the head.',
                msg_type=ros_homebot_msgs.msg.PanCentermarkChange,
                msg_condition={'state': 0},
            ))
            self.checks.append(Check(
                self,
                pre_message='Manually  re-center the head.',
                msg_type=ros_homebot_msgs.msg.PanCentermarkChange,
                msg_condition={'state': 1},
            ))
            
        # Manual and automatic pan angle targeting.
        if not self.part or self.part == 'pan_angle':
            self.checks.append(Check(
                self,
                pre_message='Turn head clockwise more than 10 degrees. Angle={angle}',
                msg_type=ros_homebot_msgs.msg.PanAngleChange,
                msg_condition={'angle__gt': 10},
                msg_update=True,
            ))
            self.checks.append(Check(
                self,
                pre_message='Center the head.',
                msg_type=ros_homebot_msgs.msg.PanAngleChange,
                msg_condition=self.is_head_roughly_center,
                msg_update=True,
                msg_quiet=True,
            ))
            self.checks.append(Check(
                self,
                pre_message='Turn head counter-clockwise more than 10 degrees. Angle={angle}',
                msg_type=ros_homebot_msgs.msg.PanAngleChange,
                msg_condition={'angle__gt': 270, 'angle__lt': 350},
                msg_update=True,
            ))
            self.checks.append(Check(
                self,
                pre_message='Center the head.',
                msg_type=ros_homebot_msgs.msg.PanAngleChange,
                msg_condition=self.is_head_roughly_center,
                msg_update=True,
                msg_quiet=True,
            ))
            self.checks.append(Check(
                self,
                pre_message='Release the head.',
                answer_options=ENTER,
            ))
            self.checks.append(Check(
                self,
                pre_message='Did the head rotate 90 degrees clockwise?',
                answer_options=YN,
                success_answer=Y,
                pre_callback=partial(self.set_head_pan_angle, 90),
            ))
            self.checks.append(Check(
                self,
                pre_message='Did the head center?',
                answer_options=YN,
                success_answer=Y,
                pre_callback=partial(self.set_head_pan_angle, 0),
            ))
            self.checks.append(Check(
                self,
                pre_message='Did the head rotate 90 degrees counter-clockwise?',
                answer_options=YN,
                success_answer=Y,
                pre_callback=partial(self.set_head_pan_angle, -90),
            ))
            self.checks.append(Check(
                self,
                pre_message='Did the head center?',
                answer_options=YN,
                success_answer=Y,
                pre_callback=partial(self.set_head_pan_angle, 0),
            ))
            self.checks.append(Check(
                self,
                pre_message='Did the head rotate 360 degrees?',
                answer_options=YN,
                success_answer=Y,
                pre_callback=self.rotate_head_360,
            ))
            
        #tilt motor
        if not self.part or self.part == 'tilt_motor':
            self.checks.append(Check(
                self,
                pre_message='Is the head tilt centered?',
                answer_options=YN,
                success_answer=Y,
                pre_callback=partial(self.tilt_head, c.TILT_CENTER),
            ))
            self.checks.append(Check(
                self,
                pre_message='Is the head tilted downwards?',
                answer_options=YN,
                success_answer=Y,
                pre_callback=partial(self.tilt_head, c.TILT_CENTER - 45),
            ))
            self.checks.append(Check(
                self,
                pre_message='Is the head tilted upwards?',
                answer_options=YN,
                success_answer=Y,
                pre_callback=partial(self.tilt_head, c.TILT_CENTER + 45),
            ))
            self.checks.append(Check(
                self,
                pre_message='Is the head tilted centered?',
                answer_options=YN,
                success_answer=Y,
                pre_callback=partial(self.tilt_head, c.TILT_CENTER),
            ))
        
        #microphones
    
    def is_head_roughly_center(self, msg):
#         sys.stdout.write('\rangle: %s ' % msg.angle); sys.stdout.flush()
        return msg.angle <= 10 or msg.angle >= 350
    
    def is_line_laser_clear(self, msg, dist_mm=100):
        line = compress_list(msg.line)
        sys.stdout.write('\rline clear: %s' % line_list_to_str(line))
        value = line[4]
        return value > 0 and value >= dist_mm
        
    def is_line_laser_obstructed(self, msg, dist_mm=100):
        line = compress_list(msg.line)
        sys.stdout.write('\rline obs: %s' % line_list_to_str(line))
        value = line[4]
        return 0 < value <= dist_mm
        
    def add_torso_checks(self):
        
#         if not self.part or self.part == 'bumper':
#             for i, pos in [(1, 'left'), (2, 'center'), (3, 'right')]:
#                 self.checks.append(Check(
#                     self,
#                     pre_message='Press the %s bumper.' % (pos),
#                     msg_type=ros_homebot_msgs.msg.BumperChange,
#                     msg_condition={'index':i, 'state':1},
#                 ))
#                 self.checks.append(Check(
#                     self,
#                     pre_message='Release the %s bumper.' % (pos),
#                     msg_type=ros_homebot_msgs.msg.BumperChange,
#                     msg_condition={'index':i, 'state':0},
#                 ))
                
        if not self.part or self.part == 'edge':
            for i, pos in [(1, 'left'), (2, 'center'), (3, 'right')]:
                self.checks.append(Check(
                    self,
                    pre_message='Obscure the %s edge sensor.' % (pos),
                    msg_type=ros_homebot_msgs.msg.EdgeChange,
                    msg_condition={'index':i, 'state':0},
                ))
                self.checks.append(Check(
                    self,
                    pre_message='Un-obscure the %s edge sensor.' % (pos),
                    msg_type=ros_homebot_msgs.msg.EdgeChange,
                    msg_condition={'index':i, 'state':1},
                ))
        
        if not self.part or self.part == 'ultrasonic':
            clear_distance_cm = 10
            sensors = [
                (1, 'left'),
                (2, 'center'),
                (3, 'right'),
            ]
            for i, pos in sensors:
                self.checks.append(Check(
                    self,
                    pre_message=(
                        'Ensure there is nothing obscuring the %s '
                        'ultrasonic sensor within %i cm. Distance={distance}') \
                            % (pos, clear_distance_cm),
                    msg_type=ros_homebot_msgs.msg.UltrasonicChange,
                    msg_condition={
                        'index': i,
                        'distance__gte': clear_distance_cm,
                    },
                    msg_update=True,
                    pre_callback=self.enable_ultrasonics,
                ))
                self.checks.append(Check(
                    self,
                    pre_message='Obscure the %s ultrasonic sensor. Distance={distance}' % (pos),
                    msg_type=ros_homebot_msgs.msg.UltrasonicChange,
                    msg_condition={
                        'index':i,
                        'distance__gt': 0, # 0=no reading
                        'distance__lte': 4,
                    },
                    msg_update=True,
                ))
                self.checks.append(Check(
                    self,
                    pre_message=(
                        'Again ensure there is nothing obscuring the %s '
                        'ultrasonic sensor within %i cm. Distance={distance}') \
                            % (pos, clear_distance_cm),
                    msg_type=ros_homebot_msgs.msg.UltrasonicChange,
                    msg_condition={
                        'index': i,
                        'distance__gte': clear_distance_cm,
                    },
                    msg_update=True,
                    post_callback=self.disable_ultrasonics,
                ))
     
        if not self.part or self.part == 'status_button':
            self.checks.append(Check(
                self,
                pre_message='Press the status button.',
                msg_type=ros_homebot_msgs.msg.StatusButtonChange,
                msg_condition={'state': 1},
            ))
            self.checks.append(Check(
                self,
                pre_message='Release the status button.',
                msg_type=ros_homebot_msgs.msg.StatusButtonChange,
                msg_condition={'state': 0},
            ))
 
        if not self.part or self.part == 'power_button':
            self.checks.append(Check(
                self,
                pre_message='Is the power button light off?',
                answer_options=YN,
                success_answer=Y,
                pre_callback=self.turn_power_button_light_off,
            ))
            self.checks.append(Check(
                self,
                pre_message='Is the power button light on?',
                answer_options=YN,
                success_answer=Y,
                pre_callback=self.turn_power_button_light_on,
                post_callback=self.reset_power_button_light,
            ))
           
        # Arduino temperature.
        if not self.part or self.part == 'arduino_temperature':
            self.checks.append(Check(
                self,
                pre_message='Wait for Arduino temperature reading...',
                msg_type=ros_homebot_msgs.msg.ArduinoTemperatureChange,
                msg_condition={'temperature__gte': 20, 'temperature__lte': 50},
                msg_timeout=100,
            ))
           
        # Battery voltage.
        if not self.part or self.part == 'battery_voltage':
            self.checks.append(Check(
                self,
                pre_message='Wait for battery voltage reading...',
                msg_type=ros_homebot_msgs.msg.BatteryVoltageChange,
                msg_condition={'voltage__gte': 6, 'voltage__lte': 15},
                msg_timeout=100,
            ))

        # External power socket

        def pre_external_power():
            self.wait_until_continue(
                'Ensure main external power is unplugged and battery is inserted.')

        if not self.part or self.part == 'external_power':
            self.checks.append(Check(
                self,
                pre_callback=pre_external_power,
                pre_message='Attach an UN-powered external power plug.',
                msg_type=ros_homebot_msgs.msg.ExternalPowerChange,
                msg_condition={'state1': 0, 'state2': 1},
            ))
            self.checks.append(Check(
                self,
                pre_message='Attach a powered external power plug.',
                msg_type=ros_homebot_msgs.msg.ExternalPowerChange,
                msg_condition={'state1': 1, 'state2': 1},
            ))
            self.checks.append(Check(
                self,
                pre_message='Remove the external power plug.',
                msg_type=ros_homebot_msgs.msg.ExternalPowerChange,
                msg_condition={'state1': 0, 'state2': 0},
            ))

        # Motor forward
        
        def pre_motor():
            self.wait_until_continue('The motors will now be tested. '
                'Ensure main power is enabled. '
                'USB power will not be sufficient.')
            self.go_forwards()
        
        if not self.part or self.part == 'motors':
            self.checks.append(Check(
                self,
                pre_message='Are both treads running forwards?',
                answer_options=YN,
                success_answer=Y,
                pre_callback=pre_motor,
            ))
            self.checks.append(Check(
                self,
                pre_message='Are both treads running backwards?',
                answer_options=YN,
                success_answer=Y,
                pre_callback=self.go_backwards,
            ))
            self.checks.append(Check(
                self,
                pre_message='Is the left motor turning backward and right motor turning forward?',
                answer_options=YN,
                success_answer=Y,
                pre_callback=self.go_left,
            ))
            self.checks.append(Check(
                self,
                pre_message='Is the left motor turning forward and right motor turning backward?',
                answer_options=YN,
                success_answer=Y,
                pre_callback=self.go_right,
                post_callback=self.stop,
            ))
            
        if not self.part or self.part == 'imu_euler':
            
            self.checks.append(Check(
                self,
                pre_message='Place upright.',
                msg_type=ros_homebot_msgs.msg.ImuEulerChange,
                msg_condition={'y__lte': 10, 'y__gte': -10, 'z__lte': 10, 'z__gte': -10},
                msg_timeout=100,
            ))
            
            self.checks.append(Check(
                self,
                pre_message='Lean to the left.',
                msg_type=ros_homebot_msgs.msg.ImuEulerChange,
                msg_condition={'y__lte': -45},
                msg_timeout=100,
            ))
            
            self.checks.append(Check(
                self,
                pre_message='Place upright.',
                msg_type=ros_homebot_msgs.msg.ImuEulerChange,
                msg_condition={'y__lte': 10, 'y__gte': -10, 'z__lte': 10, 'z__gte': -10},
                msg_timeout=100,
            ))
            
            self.checks.append(Check(
                self,
                pre_message='Lean to the right.',
                msg_type=ros_homebot_msgs.msg.ImuEulerChange,
                msg_condition={'y__gte': 45},
                msg_timeout=100,
            ))
            
            self.checks.append(Check(
                self,
                pre_message='Place upright.',
                msg_type=ros_homebot_msgs.msg.ImuEulerChange,
                msg_condition={'y__lte': 10, 'y__gte': -10, 'z__lte': 10, 'z__gte': -10},
                msg_timeout=100,
            ))
            
            self.checks.append(Check(
                self,
                pre_message='Lean forward.',
                msg_type=ros_homebot_msgs.msg.ImuEulerChange,
                msg_condition={'z__lte': -45},
                msg_timeout=100,
            ))
            
            self.checks.append(Check(
                self,
                pre_message='Place upright.',
                msg_type=ros_homebot_msgs.msg.ImuEulerChange,
                msg_condition={'y__lte': 10, 'y__gte': -10, 'z__lte': 10, 'z__gte': -10},
                msg_timeout=100,
            ))
            
            self.checks.append(Check(
                self,
                pre_message='Lean backward.',
                msg_type=ros_homebot_msgs.msg.ImuEulerChange,
                msg_condition={'z__gte': 45},
                msg_timeout=100,
            ))
    
    def stop(self):
        get_service_proxy(c.TORSO, c.ID_MOTOR_SPEED)(left=0, right=0)
    
    def go_backwards(self, speed=c.MOTOR_DEFAULT_SPEED):
        get_service_proxy(c.TORSO, c.ID_MOTOR_SPEED)(left=-speed, right=-speed)
    
    def go_forwards(self, speed=c.MOTOR_DEFAULT_SPEED):
        get_service_proxy(c.TORSO, c.ID_MOTOR_SPEED)(left=speed, right=speed)
    
    def go_left(self, speed=c.MOTOR_DEFAULT_SPEED):
        get_service_proxy(c.TORSO, c.ID_MOTOR_SPEED)(left=+speed, right=-speed)
     
    def go_right(self, speed=c.MOTOR_DEFAULT_SPEED):
        get_service_proxy(c.TORSO, c.ID_MOTOR_SPEED)(left=-speed, right=+speed)
        
    def turn_power_button_light_off(self):
        get_service_proxy(c.TORSO, c.ID_LED_AUTO)(0)
        get_service_proxy(c.TORSO, c.ID_LED)(0)
    
    def turn_power_button_light_on(self):
        get_service_proxy(c.TORSO, c.ID_LED_AUTO)(0)
        get_service_proxy(c.TORSO, c.ID_LED)(1)
    
    def reset_power_button_light(self):
        get_service_proxy(c.TORSO, c.ID_LED_AUTO)(1)
        
    def _subscribe(self, packet_id, device):
        assert device in (c.HEAD, c.TORSO)
        msg_type = packet_to_message_type(packet_id)
        self.handlers[msg_type] = MessageHandler()
        subscribe_to_topic(device, packet_id, self.record_topic)
        
    def subscribe_head(self, packet_id):
        self._subscribe(packet_id, c.HEAD)
        
    def subscribe_torso(self, packet_id):
        self._subscribe(packet_id, c.TORSO)
        

    def enable_ultrasonics(self):
        get_service_proxy(c.TORSO, c.ID_SONAR_POWER)(1)
    
    def disable_ultrasonics(self):
        get_service_proxy(c.TORSO, c.ID_SONAR_POWER)(0)

    def set_head_pan_angle(self, angle):
        angle = angle % 360
        assert 0 <= angle <= 360
        get_service_proxy(c.HEAD, c.ID_PAN_ANGLE)(angle)
        
    def tilt_head(self, angle):
        assert c.TILT_MIN <= angle <= c.TILT_MAX
        get_service_proxy(c.HEAD, c.ID_TILT_ANGLE)(angle)
    
    def rotate_head_360(self):
        self.set_head_pan_angle(120)
        time.sleep(1)
        self.set_head_pan_angle(240)
        time.sleep(1)
        self.set_head_pan_angle(360)
    
    def record_topic(self, msg):
        self.handlers[type(msg)](msg)

    def show_camera_window(self):
        
        def show():
            sys.stdout = open(os.devnull)
            os.system(
                'rosrun image_view image_view image:=/raspicam/image '
                '_image_transport:=compressed >/dev/null 2>&1')
        
        self.raspicam_process = multiprocessing.Process(target=show)
        self.raspicam_process.daemon = True
        self.raspicam_process.start()

    def hide_camera_window(self):
        if self.raspicam_process:
            #TODO:fix? leaves a child zombie?
            self.raspicam_process.terminate()
            os.system('pkill -f image_view')
            time.sleep(1)
            self.raspicam_process = None
        
    def wait_until_continue(self, message):
        raw_input(message + ' <enter>')

    def wait_until_yes(self, message):
        if raw_input(message + ' <y/n>')[:-1].lower() != 'y':
            sys.exit()
    
    def shutdown(self):
        print('Shutting down...')
        self.running = False
        #self.cancel_all() # hangs indefinitely?
        print('Done.')

if __name__ == '__main__':
    #http://wiki.ros.org/Parameter%20Server#Private_Parameters
    rospy.init_node('diagnostic')
    server = Diagnostic(rospy.get_name())
    #rospy.spin()
