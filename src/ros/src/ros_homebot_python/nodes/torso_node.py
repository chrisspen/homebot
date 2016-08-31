#!/usr/bin/env python

from ros_homebot_python import constants as c
from ros_homebot_python.node import BaseArduinoNode

class TorsoNode(BaseArduinoNode):
    
    name = c.NAME_TORSO
           
if __name__ == '__main__':
    speed = 38400
    #speed = 115200
    TorsoNode(speed=speed)
