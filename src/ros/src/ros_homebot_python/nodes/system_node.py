#!/usr/bin/env python

import time
from datetime import datetime, timedelta
from commands import getoutput

import rospy
import std_srvs.srv

import numpy as np
from scipy.stats import linregress
import humanize

from ros_homebot_msgs import msg as msgs
from ros_homebot_python import constants as c

def to_float(v):
    try:
        return float(v)
    except (ValueError, TypeError):
        return -1

def to_percent(s):
    s = s.strip()
    if not s:
        return -1
    if s.endswith(r'%'):
        s = s[:-1]
    return to_float(s)

def to_gbytes(s):
    s = s.strip()
    if not s:
        return -1
    elif s.endswith('G'):
        return float(s[:-1])
    elif s.endswith('M'):
        return float(s[:-1])*1024
    elif s.endswith('k'):
        return float(s[:-1])*1024*1024
    else:
        raise NotImplementedError('Unknown gigabytes value: %s' % s)

class SystemNode():
    """
    Reports system metrics such as CPU, memory and disk usage.
    """
    def __init__(self):
        rospy.init_node('system_node', log_level=rospy.DEBUG)
        
        self.cpu_publisher = rospy.Publisher('~cpu', msgs.CPUUsage, queue_size=1)
        self.memory_publisher = rospy.Publisher('~memory', msgs.MemoryUsage, queue_size=1)
        self.disk_publisher = rospy.Publisher('~disk', msgs.DiskUsage, queue_size=1)
        
        self.drive_path = '/dev/root'
        
        self.rate = 1#float(rospy.get_param("~rate", 1)) # hertz
        
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            
            # Find CPU.
            cpu_usage_percent = to_percent(getoutput("grep 'cpu ' /proc/stat | awk '{usage=($2+$4)*100/($2+$4+$5)} END {print usage }'"))
            
            msg = msgs.CPUUsage()
            msg.header.stamp = rospy.Time.now()
            msg.percent_used = cpu_usage_percent
#             print msg
            self.cpu_publisher.publish(msg)
            
            # Find memory.
            memory_usage_free_gbytes = to_float(getoutput("free -m|grep -i 'Mem:'|awk '{print $4}'"))
            memory_usage_used_gbytes = to_float(getoutput("free -m|grep -i 'Mem:'|awk '{print $3}'"))
            memory_usage_total_gbytes = to_float(getoutput("free -m|grep -i 'Mem:'|awk '{print $2}'"))
            memory_usage_percent = -1
            if memory_usage_used_gbytes != -1 and memory_usage_total_gbytes != -1:
                memory_usage_percent = memory_usage_used_gbytes/memory_usage_total_gbytes*100
                
            msg = msgs.MemoryUsage()
            msg.header.stamp = rospy.Time.now()
            msg.used_gbytes = memory_usage_used_gbytes
            msg.free_gbytes = memory_usage_free_gbytes
            msg.total_gbytes = memory_usage_total_gbytes
            msg.percent_used = memory_usage_percent
#             print msg
            self.memory_publisher.publish(msg)
            
            # Find disk.
            disk_usage_percent = to_percent(getoutput("df -H | grep -i "+self.drive_path+" | awk '{print $5}'"))
            disk_usage_free_gbytes = to_gbytes(getoutput("df -H | grep -i "+self.drive_path+" | awk '{print $4}'"))
            disk_usage_used_gbytes = to_gbytes(getoutput("df -H | grep -i "+self.drive_path+" | awk '{print $3}'"))
            disk_usage_total_gbytes = to_gbytes(getoutput("df -H | grep -i "+self.drive_path+" | awk '{print $2}'"))
            
            msg = msgs.DiskUsage()
            msg.header.stamp = rospy.Time.now()
            msg.used_gbytes = disk_usage_used_gbytes
            msg.free_gbytes = disk_usage_free_gbytes
            msg.total_gbytes = disk_usage_total_gbytes
            msg.percent_used = disk_usage_percent
#             print msg
            self.disk_publisher.publish(msg)
            
            r.sleep()

if __name__ == '__main__':
    SystemNode()
