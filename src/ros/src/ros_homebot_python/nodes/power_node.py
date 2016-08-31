#!/usr/bin/env python

import time
from datetime import datetime, timedelta

import rospy
import std_srvs.srv

import numpy as np
from scipy.stats import linregress
import humanize

from ros_homebot_msgs import msg as msgs
from ros_homebot_python import constants as c
from ros_homebot_python.packet import Packet

class PowerNode():
    """
    Monitors battery levels and publishes metrics.
    """
    def __init__(self):
        rospy.init_node('power_node', log_level=rospy.DEBUG)
        
        #TODO:subscribe to charge-ratio specific event?
        rospy.Subscriber("/torso_arduino/packet_read", msgs.PacketRead, self.packet_read_callback)
        
        self.remaining_time_pub = rospy.Publisher('~remaining_time', msgs.RemainingTime, queue_size=1)
        
        self.first_dt = None
        self.data_points_x = []
        self.data_points_y = []
        self.max_data_points = 1000
        
        # Moving average of the estimated remaining seconds until total loss of power.
        self.ma = None
        self.ma_ratio = 0.85
        
        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            r.sleep()

    def packet_read_callback(self, msg):
        packet = Packet.from_ros_message(msg)
#         print 'packet:', packet.id_name
        if packet.id == c.ID_GET_VALUE and packet.data[0] == c.ID_BATTERY_CHARGE_RATIO:
            
            # Calculate x.
            dt = datetime.now()
            if self.first_dt is None:
                self.first_dt = dt
            
            # Calculate y.
            charge_ratio = float(packet.parameters[1])
            print 'charge_ratio:', charge_ratio
                
            self.data_points_x.append((dt - self.first_dt).total_seconds())
            self.data_points_y.append(charge_ratio)
            if len(self.data_points_x) > self.max_data_points:
                self.data_points_x = self.data_points_x[1:]
                self.data_points_y = self.data_points_y[1:]
                
            if len(self.data_points_x) > 60:
                
                # Calculate linear regression estimate.
                m, b, r_value, p_value, std_err = linregress(self.data_points_x, self.data_points_y)
                print 'linregress:', m, b, r_value, p_value, std_err
                # y = mx+b => (y-b)/m = x => -b/m = x 
                x_at_zero = -b/m
                
                # Ignore if it's junk because there's not enough data.
                remaining_seconds = x_at_zero - self.data_points_x[-1]
                print 'remaining_seconds:', remaining_seconds
                if remaining_seconds < 0:
                    return
                
                # Perform a moving average over that, to smooth out swings.
                _ma = self.ma
                if self.ma is None:
                    self.ma = remaining_seconds
                else:
                    self.ma = self.ma * self.ma_ratio + remaining_seconds * (1 - self.ma_ratio)
                
                _td = -timedelta(seconds=self.ma)
                print 'optimist: charge will end in %s' % humanize.naturaltime(_td)
                
                # Clamp down to find the most likely worst-case scenario.
                if _ma is not None and len(self.data_points_x) > 100:
                    self.ma = min(_ma, self.ma)
                _td = -timedelta(seconds=self.ma)
                print 'pessimist: charge will end in %s' % humanize.naturaltime(_td)
                
                msg = msgs.RemainingTime()
                msg.header.stamp = rospy.Time.now()
                msg.remaining_seconds = rospy.Duration(secs=self.ma)
                msg.error = std_err
                self.remaining_time_pub.publish(msg)

if __name__ == '__main__':
    PowerNode()
