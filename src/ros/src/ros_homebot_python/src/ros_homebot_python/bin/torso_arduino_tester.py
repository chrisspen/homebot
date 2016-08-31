#!/usr/bin/env python
"""
2015.12.19 CKS
Script for manually testing and verifying functions controlled via the torso Arduino.
"""
import os
import sys

import serial

BASE_DIR = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(BASE_DIR, '..'))
sys.path.insert(0, os.path.join(BASE_DIR, '../..'))

from ros_homebot_python import constants as c
from ros_homebot_python import ui
from ros_homebot_python.packet import Packet
from ros_homebot_python import utils

class UI(ui.BaseUI):
    
    title = 'Homebot Torso Arduino Tester (press q to quit)'
    
    identity = c.NAME_TORSO
    
    def get_controls(self):
        lst = super(UI, self).get_controls()
        
        self.led_auto = ctrl = ui.Control(
            label='LED Auto',
            default=True,
            choices=(False, True),
            choice_map={True: 'on', False: 'off'},
            on_change=self.on_change_led_auto,
            auto_get=True,
            id=c.ID_LED_AUTO,
        )
        lst.append(ctrl)
        
        self.go_to_sleep = ctrl = ui.Control(
            label='Go To Sleep',
            default=False,
            choices=(False, True),
            choice_map={True: 'on', False: 'off'},
            on_change=self.on_change_go_to_sleep,
        )
        lst.append(ctrl)
        
        self.go_to_sleep = ctrl = ui.Control(
            label='Shutdown',
            default=False,
            choices=(False, True),
            choice_map={True: 'on', False: 'off'},
            on_change=self.on_change_shutdown,
        )
        lst.append(ctrl)
        
        self.motors = ctrl = ui.Control(
            label='Motors',
            default=c.MOTOR_BREAK,
            choices=(
                c.MOTOR_BREAK,
                c.MOTOR_FORWARD,
                c.MOTOR_REVERSE,
                c.MOTOR_TURN_CW,
                c.MOTOR_TURN_CCW,
            ),
            #choice_map={True: 'on', False: 'off'},
            input_map={
                'up': c.MOTOR_FORWARD,
                'down': c.MOTOR_REVERSE,
                'left': c.MOTOR_TURN_CCW,
                'right': c.MOTOR_TURN_CW,
                's': c.MOTOR_BREAK,
            },
            on_change=self.on_change_motors,
        )
        lst.append(ctrl)
        
        ctrl = ui.Control(
            label='Battery Voltage',
            #default=,
            editable=False,
            auto_get=True,
            id=c.ID_BATTERY_VOLTAGE,
        )
        lst.append(ctrl)
        
        ctrl = ui.Control(
            label='Battery Charge',
            #default=,
            editable=False,
            auto_get=True,
            id=c.ID_BATTERY_CHARGE_RATIO,
        )
        lst.append(ctrl)
        
        ctrl = ui.Control(
            label='Battery Temperature',
            #default=,
            editable=False,
            #poll_seconds=1,
            #on_change=self.on_centermark_change,
            auto_get=True,
            id=c.ID_BATTERY_TEMP,
        )
        lst.append(ctrl)
        
        ctrl = ui.Control(
            label='Arduino Temperature',
            #default=,
            editable=False,
            #poll_seconds=1,
            #on_change=self.on_centermark_change,
            auto_get=True,
            id=c.ID_ARDUINO_TEMP,
        )
        lst.append(ctrl)
        
        self.external_power = ctrl = ui.Control(
            label='External Power',
            editable=False,
            #default=True,
            choices=(False, True),
            choice_map={True: 'on', False: 'off'},
            #on_change=self.on_change_led_auto,
            auto_get=True,
            id=c.ID_EXTERNAL_POWER,
        )
        lst.append(ctrl)
        
        self.accelgryo = ctrl = ui.Control(
            label='Accel/Gryo',
            editable=False,
            default='',
            #choices=(False, True),
            #choice_map={True: 'on', False: 'off'},
            #on_change=self.on_change_led_auto,
            auto_get=True,
            id=c.ID_ACCELGYRO,
            on_set_value=self.on_set_value_accelgyro,
        )
        lst.append(ctrl)
        
        for _i in xrange(1, 4):
            ctrl = ui.Control(
                label='Bumper %i' % _i,
                #default=,
                editable=False,
                #poll_seconds=1,
                #on_change=self.on_centermark_change,
                auto_get=True,
                id=c.ID_BUMPER+str(_i),
            )
            lst.append(ctrl)
        
        self.ultrasonic_power = ctrl = ui.Control(
            label='Ultrasonic Power',
            default=False,
            choices=(False, True),
            choice_map={True: 'on', False: 'off'},
            auto_get=True,
            id=c.ID_SONAR_POWER,
            on_change=self.on_change_ultrasonic_power,
            on_set_value=self.on_set_value_ultrasonic_power,
        )
        lst.append(ctrl)
        for _i in xrange(1, 4):
            ctrl = ui.Control(
                label='Ultrasonic %i' % _i,
                #default=,
                editable=False,
                #poll_seconds=1,
                #on_change=self.on_centermark_change,
                auto_get=True,
                id=c.ID_ULTRASONIC+str(_i),
            )
            lst.append(ctrl)
        
        for _i in xrange(1, 4):
            ctrl = ui.Control(
                label='Edge %i' % _i,
                #default=,
                editable=False,
                #poll_seconds=1,
                #on_change=self.on_centermark_change,
                auto_get=True,
                id=c.ID_EDGE+str(_i),
            )
            lst.append(ctrl)
        
        return lst

    def on_set_value_ultrasonic_power(self, ctrl, *values):
        #self.log(' q C on_set_value_ultrasonic_power:', values)
        v = bool(int(values[0]))
        #v = ctrl.choice_map[v]
        return v
            
    def on_set_value_accelgyro(self, ctrl, *values):
        return ' '.join(map(str, values))

    def on_change_led_auto(self, value):
        self.command_queue.put(Packet(c.ID_LED_AUTO, utils.to_10(value)))

    def on_change_ultrasonic_power(self, value):
        self.command_queue.put(Packet(c.ID_SONAR_POWER, utils.to_10(value)))

#     def on_response_led_auto(self, data):
#         pass #TODO:update control?
#         
    def on_change_motors(self, value):
        speed = 200
        if value == c.MOTOR_BREAK: # s
            self.command_queue.put(Packet(c.ID_MOTOR_SPEED, '0 0'))
        elif value == c.MOTOR_FORWARD:
            self.command_queue.put(Packet(c.ID_MOTOR_SPEED, '{speed} {speed}'.format(speed=speed)))
        elif value == c.MOTOR_REVERSE:
            self.command_queue.put(Packet(c.ID_MOTOR_SPEED, '-{speed} -{speed}'.format(speed=speed)))
        elif value == c.MOTOR_TURN_CW:
            self.command_queue.put(Packet(c.ID_MOTOR_SPEED, '{speed} -{speed}'.format(speed=speed)))
        elif value == c.MOTOR_TURN_CCW:
            self.command_queue.put(Packet(c.ID_MOTOR_SPEED, '-{speed} {speed}'.format(speed=speed)))
         
    def on_change_go_to_sleep(self, value):
        #self.command_queue.put(Packet(c.ID_GO_TO_SLEEP, 10000)) # 10 seconds
        self.command_queue.put(Packet(c.ID_GO_TO_SLEEP, 30000)) # 30 seconds
        #self.command_queue.put(Packet(c.ID_GO_TO_SLEEP, 60000)) # 1 minute
        #self.command_queue.put(Packet(c.ID_GO_TO_SLEEP, 300000)) # 5 minutes
    
    def on_change_shutdown(self, value):
        self.command_queue.put(Packet(c.ID_SHUTDOWN))
    
    def on_response_motors(self, data):
        pass
        
if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Test the Homebot torso Arduino.')
    parser.add_argument('--port', default=None,
        help='The port to use when connecting to the Arduino.')
    parser.add_argument('--speed', default='57600',
        help='The baudrate to use when connecting to the Arduino.')
    
    args = parser.parse_args()
    
    ui = UI(**args.__dict__)
    ui.run()
