#!/usr/bin/env python

import os
import time
import sys
from threading import Thread

import rospy

from ros_homebot_msgs import msg as msgs

from django.core.wsgi import get_wsgi_application

from ros_homebot_python import constants as c
from ros_homebot_python.node import (
    packet_to_service_type,
    packet_to_service_request_type,
    get_topic_name,
    subscribe_to_topic,
    camel_to_underscore,
)

from homebot_dashboard.server import Server
from homebot_dashboard import sockets

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DJANGO_DIR = os.path.abspath(os.path.join(BASE_DIR, '../src'))
sys.path.insert(0, DJANGO_DIR)
os.environ.setdefault("DJANGO_SETTINGS_MODULE", "settings")

# Fixes error "Apps aren't loaded yet."
application = get_wsgi_application()

class ServerThread(Thread):
    
    server = None
    
    daemon = True
    
    def run(self):
        self.server = Server(use_reloader=True)
        self.server.run()
        
    def stop(self):
        self.server.stop()

class HomebotTeleopROS():
    
    def __init__(self):
        rospy.init_node('homebot_teleop', log_level=rospy.DEBUG)
        
        self.verbose = int(rospy.get_param("~verbose", 0))
        
        self.rate = int(rospy.get_param("~rate", 50))
        r = rospy.Rate(self.rate)
        
        subscribe_to_topic(c.HEAD, c.ID_ARDUINO_TEMP, self.packet_read_callback)
        subscribe_to_topic(c.HEAD, c.ID_PAN_ANGLE, self.packet_read_callback)
        subscribe_to_topic(c.HEAD, c.ID_PONG, self.packet_read_callback)
        
        #subscribe_to_topic(c.TORSO, c.ID_ACCELGYRO, self.packet_read_callback)
        subscribe_to_topic(c.TORSO, c.ID_ARDUINO_TEMP, self.packet_read_callback)
        subscribe_to_topic(c.TORSO, c.ID_BATTERY_CHARGE_RATIO, self.packet_read_callback)
        subscribe_to_topic(c.TORSO, c.ID_BATTERY_TEMP, self.packet_read_callback)
        subscribe_to_topic(c.TORSO, c.ID_BATTERY_VOLTAGE, self.packet_read_callback)
        subscribe_to_topic(c.TORSO, c.ID_BUMPER, self.packet_read_callback)
        subscribe_to_topic(c.TORSO, c.ID_EDGE, self.packet_read_callback)
        subscribe_to_topic(c.TORSO, c.ID_EXTERNAL_POWER, self.packet_read_callback)
        subscribe_to_topic(c.TORSO, c.ID_PONG, self.packet_read_callback)
        subscribe_to_topic(c.TORSO, c.ID_ULTRASONIC, self.packet_read_callback)
        
        rospy.Subscriber('/system_node/cpu', msgs.CPUUsage, self.packet_read_callback)
        rospy.Subscriber('/system_node/disk', msgs.DiskUsage, self.packet_read_callback)
        rospy.Subscriber('/system_node/memory', msgs.MemoryUsage, self.packet_read_callback)
        
        self.server_thread = ServerThread()
        self.server_thread.start()

        # Cleanup when termniating the node
        rospy.on_shutdown(self.shutdown)
        
        # Start polling the sensors and base controller
        while not rospy.is_shutdown():
            
            # Publish all sensor values on a single topic for convenience
#             now = rospy.Time.now()
#             
#             r.sleep()
            time.sleep(1)
    
    def loginfo(self, *msg):
        if self.verbose:
            rospy.loginfo(' '.join(map(str, msg)))
    
    def publish_packet_write(self, device, packet_id, *args):
        
        # Lookup service name.
        #service_name = '/%s_arduino/%s' % (device.lower(), get_name(packet_id))
        service_name = get_topic_name(device, packet_id)
        
        # Lookup service type.
        service_type = packet_to_service_type(packet_id)
        service_req_type = packet_to_service_request_type(packet_id)
        
        # Create request.
        kwargs = {}
        for arg_name, arg_value in zip(service_req_type.__slots__, args):
            kwargs[arg_name] = arg_value
        
        # Send request.
        print 'sending request:', service_name, service_type, kwargs
        rospy.ServiceProxy(service_name, service_type)(**kwargs)
    
    def packet_read_callback(self, msg):
        b = sockets.get_broadcaster()
        connections = sockets.get_connections()
        #self.loginfo('packet_read_callback.b: %s' % b)
#         print 'received:', msg
#         print 'connections:', connections, b
        if b and connections:
            sockets.node = self
            
            self.loginfo('Received message: %s' % msg)
            if 'device' in msg.__slots__:
                device_name = c.INDEX_TO_NAME[msg.device].lower()
                event_name = '%s_%s' % (device_name, camel_to_underscore(type(msg).__name__))
            else:
                event_name = camel_to_underscore(type(msg).__name__)
            data = {}
            for k in msg.__slots__:
#                 print 'k:', k, getattr(msg, k)
                if k == 'device' or k == 'frame_id' or k == 'header':
                    continue
                data[k] = getattr(msg, k)
#             print 'broadcasting:', event_name
#             print 'data:', str(data)
            b.broadcast_event(event_name, data)
    
    def shutdown(self):
        self.loginfo("Stopping the Homebot teleop SocketIO server...")
        self.server_thread.stop()
        self.server_thread.join()
        
if __name__ == '__main__':
    HomebotTeleopROS()
