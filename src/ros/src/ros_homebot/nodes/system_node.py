#!/usr/bin/env python

import time
from datetime import datetime, timedelta
from commands import getoutput

import rospy
import std_srvs.srv
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import numpy as np
from scipy.stats import linregress
import humanize

from ros_homebot_msgs import msg as msgs
from ros_homebot_python import constants as c

OK = DiagnosticStatus.OK
WARN = DiagnosticStatus.WARN
ERROR = DiagnosticStatus.ERROR
STALE = DiagnosticStatus.STALE

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

def get_cpu_temp():
    """
    In general you should consider 60 degrees Celcius the absolute maximum for long periods,
    but aim for 45-50 degrees to be safe.
    
    Readings are in thousandths of degrees Celcius (although in older kernels, it may have 
    just been degrees C).
    """
    degrees_celcius = int(getoutput('cat /sys/class/thermal/thermal_zone0/temp').strip())/1000.
    return degrees_celcius

class SystemNode():
    """
    Reports system metrics such as CPU, memory and disk usage.
    """
    def __init__(self):
        rospy.init_node('system_node', log_level=rospy.DEBUG)
        
        self.cpu_publisher = rospy.Publisher('~cpu', msgs.CPUUsage, queue_size=1)
        self.memory_publisher = rospy.Publisher('~memory', msgs.MemoryUsage, queue_size=1)
        self.disk_publisher = rospy.Publisher('~disk', msgs.DiskUsage, queue_size=1)
        self.diagnostics_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)
        
        self.drive_path = '/dev/root'
        
        self.rate = 1#float(rospy.get_param("~rate", 1)) # hertz
        
        rospy.loginfo('Ready')
        
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            
            # Find CPU.
#             rospy.loginfo('Finding CPU.')
            cpu_usage_percent = to_percent(getoutput(
                "grep 'cpu ' /proc/stat | "
                "awk '{usage=($2+$4)*100/($2+$4+$5)} END {print usage }'"))
            cpu_usage_percent_level = OK
            if cpu_usage_percent >= c.CPU_USAGE_PERCENT_ERROR:
                cpu_usage_percent_level = ERROR
            elif cpu_usage_percent >= c.CPU_USAGE_PERCENT_WARN:
                cpu_usage_percent_level = WARN
            
            msg = msgs.CPUUsage()
            msg.header.stamp = rospy.Time.now()
            msg.percent_used = cpu_usage_percent
#             print msg
            self.cpu_publisher.publish(msg)
            
            # Find memory.
            memory_usage_free_gbytes = to_float(getoutput(
                "free -m|grep -i 'Mem:'|awk '{print $4}'"))
            memory_usage_used_gbytes = to_float(getoutput(
                "free -m|grep -i 'Mem:'|awk '{print $3}'"))
            memory_usage_total_gbytes = to_float(getoutput(
                "free -m|grep -i 'Mem:'|awk '{print $2}'"))
            memory_usage_percent = -1
            if memory_usage_used_gbytes != -1 and memory_usage_total_gbytes != -1:
                memory_usage_percent = memory_usage_used_gbytes/memory_usage_total_gbytes*100
            memory_usage_percent_level = OK
            if memory_usage_percent >= c.MEMORY_USAGE_PERCENT_ERROR:
                memory_usage_percent_level = ERROR
            elif memory_usage_percent >= c.MEMORY_USAGE_PERCENT_WARN:
                memory_usage_percent_level = WARN
                
            msg = msgs.MemoryUsage()
            msg.header.stamp = rospy.Time.now()
            msg.used_gbytes = memory_usage_used_gbytes
            msg.free_gbytes = memory_usage_free_gbytes
            msg.total_gbytes = memory_usage_total_gbytes
            msg.percent_used = memory_usage_percent
#             print msg
            self.memory_publisher.publish(msg)
            
            # Find disk.
            disk_usage_percent = to_percent(getoutput(
                "df -H | grep -i "+self.drive_path+" | awk '{print $5}'"))
            disk_usage_free_gbytes = to_gbytes(getoutput(
                "df -H | grep -i "+self.drive_path+" | awk '{print $4}'"))
            disk_usage_used_gbytes = to_gbytes(getoutput(
                "df -H | grep -i "+self.drive_path+" | awk '{print $3}'"))
            disk_usage_total_gbytes = to_gbytes(getoutput(
                "df -H | grep -i "+self.drive_path+" | awk '{print $2}'"))
            disk_usage_level = OK
            if disk_usage_percent >= c.DISK_USAGE_PERCENT_ERROR:
                disk_usage_level = ERROR
            if disk_usage_percent >= c.DISK_USAGE_PERCENT_WARN:
                disk_usage_level = WARN
            
            msg = msgs.DiskUsage()
            msg.header.stamp = rospy.Time.now()
            msg.used_gbytes = disk_usage_used_gbytes
            msg.free_gbytes = disk_usage_free_gbytes
            msg.total_gbytes = disk_usage_total_gbytes
            msg.percent_used = disk_usage_percent
#             print msg
            self.disk_publisher.publish(msg)

            # Find CPU temperature.
            cpu_temp = get_cpu_temp()
            cpu_temp_level = OK
            if cpu_temp >= c.CPU_TEMP_ERROR:
                cpu_temp_level = ERROR
            elif cpu_temp >= c.CPU_TEMP_WARN:
                cpu_temp_level = WARN
            
            # Publish standard diagnostics.
            array = DiagnosticArray()
            cpu_temperature_status = DiagnosticStatus(
                name='CPU Temperature',
                level=cpu_temp_level)
            cpu_temperature_status.values = [
                KeyValue(key='celcius', value=str(cpu_temp)),
            ]
            cpu_usage_status = DiagnosticStatus(
                name='CPU Usage',
                level=cpu_usage_percent_level)
            cpu_usage_status.values = [
                KeyValue(key='percent', value=str(cpu_usage_percent)),
            ]
            disk_usage_status = DiagnosticStatus(
                name='Disk Usage',
                level=disk_usage_level)
            disk_usage_status.values = [
                KeyValue(key='percent', value=str(disk_usage_percent)),
                KeyValue(key='free gb', value=str(disk_usage_free_gbytes)),
                KeyValue(key='used gb', value=str(disk_usage_used_gbytes)),
                KeyValue(key='total gb', value=str(disk_usage_total_gbytes)),
            ]
            memory_usage_status = DiagnosticStatus(
                name='Memory Usage',
                level=memory_usage_percent_level)
            memory_usage_status.values = [
                KeyValue(key='percent', value=str(memory_usage_percent)),
                KeyValue(key='free gb', value=str(memory_usage_free_gbytes)),
                KeyValue(key='used gb', value=str(memory_usage_used_gbytes)),
                KeyValue(key='total gb', value=str(memory_usage_total_gbytes)),
            ]
            array.status = [
                cpu_temperature_status,
                cpu_usage_status,
                disk_usage_status,
                memory_usage_status,
            ]
            self.diagnostics_pub.publish(array)

            r.sleep()

if __name__ == '__main__':
    SystemNode()
