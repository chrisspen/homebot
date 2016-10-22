
from ros_homebot_msgs import srv as srvs
from ros_homebot_python import constants as c

class Robot(object):
    """
    A helper class to simplify interacting with ROS.
    
    Usage:
    
        robot = Robot()
        robot.enable.head_pan = True
        robot.enable.head_tilt = True
        robot.
    """
    
    enabled_topics = [
    ]
    
    def __init__(self):
        pass

    def say(self, *args, **kwargs):
        from ros_homebot_python.node import say
        say(*args, **kwargs)

    def move(self, distance, duration):
        """
        Moves the platform a given distance within a given time.
        
        distance := (x,y) in meters
        duration := scalar in seconds
        
        The positive y-axis points away from the front of the robot.
        The vector (0, 1) means forward 1 meter
        The vector (0, 0.5) means forward 0.5 meters
        The vector (0, 0) means stop.
        The vector (0, -1) means reverse 1 meter
        The vector (1, 0) means turn counter clockwise and move forward 1 meter
        The vector (-1, 0) means turn clockwise and move forward 1 meter
        """
        raise NotImplementedError
    
    def turn(self, theta, duration):
        """
        Rotates platform in place by a given angle in radians with a given duration.
        
        theta := angle in radians
        duration := scalar in seconds
        """
        raise NotImplementedError
