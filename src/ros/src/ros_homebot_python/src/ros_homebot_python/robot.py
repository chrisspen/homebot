
import rospy
# from ros_homebot_msgs import srv as srvs

from ros_homebot_python import constants as c
from ros_homebot_python.node import (
#    subscribe_to_topic,
    get_service_proxy,
#     packet_to_message_type,
#     set_line_laser,
#     say,
)

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
    
    node_name = None
    
    sleep_hertz = 60
    
    # http://wiki.ros.org/rospy/Overview/Logging
    log_level = 'ERROR' # 'DEBUG'
    
    def __init__(self):
        
        self.running = True
        
        rospy.init_node(self.node_name, log_level=getattr(rospy, self.log_level))
        
        rospy.on_shutdown(self._on_shutdown)
        
        self.init_subscriptions()
        
        r = rospy.Rate(self.sleep_hertz)
        while not rospy.is_shutdown():
            self.on_loop()
            r.sleep()

    def init_subscriptions(self):
        pass
        
    def is_shutdown(self):
        return not self.running

    def subscribe(self, *args, **kwargs):
        rospy.Subscriber(*args, **kwargs)#topic_name, msg_type, callback)

    def on_loop(self):
        pass

    def _on_shutdown(self):
        self.running = False
        self.on_shutdown()
    
    def on_shutdown(self):
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

    def set_rgbled(self, value):
        """
        Value is a 3-part tuple representing the RGB values of the color, each value being an integer in the range 0-254.
        """
        assert len(value) == 3
        for i, v in enumerate(value):
            assert 0 <= i <= 254
            get_service_proxy(c.HEAD, c.ID_LED)(i, v)
            
    def set_ultrabright(self, value):
        """
        Value is a single integer between 0-254 representing brightness with 0 being off and 254 being full brightness.
        """
        assert 0 <= value <= 254
        get_service_proxy(c.HEAD, c.ID_LED)(3, value)
