#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
http://www.robotshop.com/blog/en/drive-motor-sizing-tool-9698
"""
import sys
from math import *
#http://pint.readthedocs.org/en/0.6/tutorial.html
from pint import UnitRegistry

ureg = UnitRegistry()

def velocity_to_rpm(v, r):
    kph = v.to(kilometer/hour)
    r = r.to(kilometer)
    d = r*2
    rpm = (kph / (2*pi*r)) * ((1*hour)/(60.*minute)) * rev
    return rpm

def velocity_to_radps(v, r):
    return velocity_to_rpm(v, r).to(radian/second)

# Units
km = kilometer = ureg.kilometer
meter = ureg.meter
newton = ureg.newton
cm = centimeter = ureg.centimeter
hr = hour = ureg.hour
mm = millimeter = ureg.millimeter
rev = revolution = ureg.revolution
minute = ureg.minute
sec = second = ureg.second
kg = kilogram = ureg.kilogram
gm = gram = ureg.gram
deg = degree = ureg.degree
rad = radian = ureg.radian
oz = ureg.oz
inch = ureg.inch
watt = ureg.watt
#1 watt = 1 kg*m^2/s^3
gmcm_to_nm = 10197.1621298 * gm*cm / (newton*meter)

# Constants
A = 9.8 * meter/(sec**2) # acceleration of gravity

# Constraints
max_linear_velocity = 2.5*km/hour # half average walking speed
W = 5*kg * A # estimated_platform_weight
maximum_incline = 0*deg
maximum_pushing_force = W/4.
rolling_friction = 0.015 # rubber on pavement
N = 2 # number_of_powered_motors
wheel_diameter = (1*inch).to(mm)#120*mm

# Calculate total applied force at worst case.
Ftotal = W * (rolling_friction*cos(maximum_incline.to(radian)) + sin(maximum_incline.to(radian))) + maximum_pushing_force
print 'Ftotal:',Ftotal

# Calculate power requirement.
# P = F * v
Ptotal = (Ftotal * max_linear_velocity.to(meter/sec)).to(watt)
print 'Ptotal:', Ptotal # should be in watts = kg*m^2/s^3
Pmotor = Ptotal/N
print 'Pmotor:', Pmotor

# Calculate torque and speed requirement.
# p = ω • τ = (rad/sec) * (N*m) = watt => τ = p / ω = watt / (rad/sec)
# watt / (rad/sec) = torque = Nm = gm*cm?
max_angular_velocity = velocity_to_radps(max_linear_velocity, wheel_diameter/2)
#print 'max_angular_velocity:',max_angular_velocity # should be in rad/sec
required_rpm = max_angular_velocity / 0.104719755 * (rev/rad) * (sec/minute)
print 'Max RPM:',required_rpm
Tmotor = (Pmotor / max_angular_velocity).to(newton*meter)
print 'Tmotor: %s, %s, %s' % (Tmotor, Tmotor*gmcm_to_nm, (Tmotor*gmcm_to_nm).to(oz*inch))
