#!/usr/bin/env python
"""
2015.12.19 CKS
Script for manually testing and verifying functions controlled via the head Arduino.
"""
import os
import sys

BASE_DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(BASE_DIR, '..'))
sys.path.insert(0, os.path.join(BASE_DIR, '../..'))

from ros_homebot_python import constants as c
from ros_homebot_python import ui
from ros_homebot_python import utils
from ros_homebot_python.packet import Packet, BooleanPacket, KVPacket

class UI(ui.BaseUI):
    
    title = 'Homebot Head Arduino Tester (press q to quit)'
    
    identity = c.NAME_HEAD
    
    def get_controls(self):
        
        lst = super(UI, self).get_controls()
        
        ## Tilt
        
        self.tilt_power = ctrl = ui.Control(
            label='Tilt Power',
            default=False,
            choices=(False, True),
            choice_map={True: 'on', False: 'off'},
            on_change=self.on_tilt_power_change,
        )
        lst.append(ctrl)
        
        self.tilt_angle = ctrl = ui.Control(
            label='Tilt Angle Target',
            default=90,
            min_value=90-30,
            max_value=90+30,
            step=1,
            #debounce=DEBOUNCE,
            on_change=self.on_tilt_angle_change,
        )
        lst.append(ctrl)
        
        self.tilt_angle = ctrl = ui.Control(
            label='Tilt Angle Actual',
            editable=False,
            auto_get=True,
            id=c.ID_TILT_ANGLE,
        )
        lst.append(ctrl)
        
        # Action to start calibration.
        self.tilt_calibrate = ctrl = ui.Control(
            label='Tilt Calibrate',
            editable=False,
            default='<right arrow to activate>',
#             choices=(False, True),
#             choice_map={True: 'on', False: 'off'},
            on_enter=self.on_tilt_calibrate,
        )
        lst.append(ctrl)
        
        # Action to start centering.
        self.tilt_go_to_center = ctrl = ui.Control(
            label='Tilt Go To Center',
            editable=False,
            default='<right arrow to activate>',
#             default=False,
#             choices=(False, True),
#             choice_map={True: 'on', False: 'off'},
            on_enter=self.on_tilt_center,
        )
        lst.append(ctrl)
        
        ## Pan
        
        self.pan_power = ctrl = ui.Control(
            label='Pan Power',
            default=False,
            choices=(False, True),
            choice_map={True: 'on', False: 'off'},
            on_change=self.on_pan_power_change,
        )
        lst.append(ctrl)
        
        self.pan_angle = ctrl = ui.Control(
            label='Pan Angle Target',
            default=0,
            min_value=0,
            max_value=360-1,
            loop=True,
            step=1,
            #debounce=DEBOUNCE,
            on_change=self.on_pan_angle_change,
        )
        lst.append(ctrl)
        
        self.pan_angle = ctrl = ui.Control(
            label='Pan Angle Actual',
            editable=False,
            auto_get=True,
            id=c.ID_PAN_ANGLE,
        )
        lst.append(ctrl)
        
        # Action to start calibration.
        self.pan_calibrate = ctrl = ui.Control(
            label='Pan Calibrate',
            editable=False,
            default='<right arrow to activate>',
#             choices=(False, True),
#             choice_map={True: 'on', False: 'off'},
            on_enter=self.on_pan_calibrate,
        )
        lst.append(ctrl)
        
        # Action to start centering.
        self.pan_go_to_center = ctrl = ui.Control(
            label='Pan Go To Center',
            editable=False,
            default='<right arrow to activate>',
#             default=False,
#             choices=(False, True),
#             choice_map={True: 'on', False: 'off'},
            on_enter=self.on_pan_center,
        )
        lst.append(ctrl)
        
        #TODO:deprecated? use pan angle instead once PID controller implemented
#         self.pan_speed = ctrl = ui.Control(
#             label='Pan Speed',
#             default=0,
#             min_value=-255,
#             max_value=+255,
#             step=5,
#             on_change=self.on_pan_speed_change,
#         )
#         lst.append(ctrl)
        
        # Show if we've reached centermark.
        self.centermark = ctrl = ui.Control(
            label='Pan At Centermark',
            #default=,
            editable=False,
            #poll_seconds=1,
            #on_change=self.on_centermark_change,
            auto_get=True,
            id=c.ID_PAN_CENTERMARK,
        )
        lst.append(ctrl)
        
        # Show pan full rev count
        self.pan_full_rev = ctrl = ui.Control(
            label='Pan Full Rev Count',
            #default=,
            editable=False,
            #poll_seconds=1,
            #on_change=self.on_centermark_change,
            auto_get=True,
            id=c.ID_PAN_FULL_REV_COUNT,
        )
        lst.append(ctrl)
        
        return lst
    
    @ui.debounce(c.DEBOUNCE)
    def on_pan_power_change(self, value):
        value = utils.to_10(value)
        self.command_queue.put(BooleanPacket(c.ID_PAN_POWER, value))
    
    @ui.debounce(c.DEBOUNCE)
    def on_tilt_power_change(self, value):
        value = utils.to_10(value)
        self.command_queue.put(BooleanPacket(c.ID_TILT_POWER, value))
    
    @ui.debounce(c.DEBOUNCE)
    def on_pan_angle_change(self, value):
        self.command_queue.put(Packet(c.ID_PAN_ANGLE, value))
    
    @ui.debounce(c.DEBOUNCE)
    def on_pan_speed_change(self, value):
        self.command_queue.put(Packet(c.ID_PAN_SPEED, value))
        
    @ui.debounce(c.DEBOUNCE)
    def on_tilt_angle_change(self, value):
        self.command_queue.put(Packet(c.ID_TILT_ANGLE, value))
        
    @ui.debounce(c.DEBOUNCE)
    def on_pan_calibrate(self, value=True):
        assert value in (True, False)
        if value:
            self.command_queue.put(Packet(c.ID_CALIBRATE, c.NAME_PAN))
        
    @ui.debounce(c.DEBOUNCE)
    def on_tilt_calibrate(self, value=True):
        assert value in (True, False)
        if value:
            self.command_queue.put(Packet(c.ID_CALIBRATE, c.NAME_TILT))
            
    @ui.debounce(c.DEBOUNCE)
    def on_pan_center(self, value=True):
        assert value in (True, False)
        if value:
            self.command_queue.put(Packet(c.ID_GO_TO_CENTER, c.NAME_PAN))
            
    @ui.debounce(c.DEBOUNCE)
    def on_tilt_center(self, value=True):
        assert value in (True, False)
        if value:
            self.command_queue.put(Packet(c.ID_GO_TO_CENTER, c.NAME_TILT))
    
if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Test the Homebot head Arduino.')
    parser.add_argument('--port', default=None,
        help='The port to use when connecting to the Arduino.')
    parser.add_argument('--speed', default='57600',
        help='The baudrate to use when connecting to the Arduino.')
    
    args = parser.parse_args()
    
    ui = UI(**args.__dict__)
    ui.run()
