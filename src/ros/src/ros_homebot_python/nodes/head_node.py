#!/usr/bin/env python

from ros_homebot_python import constants as c
from ros_homebot_python.node import BaseArduinoNode

class HeadNode(BaseArduinoNode):
    
    name = c.NAME_HEAD
           
if __name__ == '__main__':
    HeadNode()
