#!../../../.env/bin/python
from math import *
import pint
ureg = pint.UnitRegistry()

def rotate(x, y, theta):
    xp = x*cos(theta) - y*sin(theta)
    yp = x*sin(theta) + y*cos(theta)
    return xp, yp

def combine_cog(m1, x1, m2, x2):
    print 'a1:',m1*x1
    print 'a2:',m2*x2
    a = (m1*x1 + m2*x2)
    b = (m1 + m2)
    return a/b

mm = ureg.mm
kg = ureg.kg
g = ureg.gram
sec = ureg.second
radians = ureg.radians
degrees = ureg.degrees

width = 65*mm
height = 300*mm
mass = 1300*g
velocity = 40*mm/sec
depression = 10*mm

imperical_maximum_angle = atan(20/65.)*radians
print 'static maximum angle:', imperical_maximum_angle.to('degrees')

momentum = mass*velocity
print 'momentum:', momentum

body_cog = 0*mm, height*0.75

counterweight = 20*9*g
print 'counterweight:', counterweight
counterweight_coords = width/2, height

approx_center = (width/2, height/2)

#theta = atan(o/a), o=width, a=height
maximum_angle = atan(width*.5/body_cog[1])*radians
print 'maximum_angle:', maximum_angle.to('degrees')

#theta = asin(y1/x0), y1=depression, x0=width
depression_angle = asin(depression/width)*radians
print 'depression_angle:', depression_angle.to('degrees')

if depression_angle > maximum_angle:
    print 'system unbalanced!'

combined_cog = (
    combine_cog(m1=mass, x1=body_cog[0], m2=counterweight, x2=counterweight_coords[0]),
    combine_cog(m1=mass, x1=body_cog[1], m2=counterweight, x2=counterweight_coords[1]),
)
print 'old cog:', body_cog
print 'new cog:', combined_cog

#f=m*a

new_maximum_angle = atan((width*.5+combined_cog[0])/combined_cog[1])*radians
print 'new_maximum_angle:', new_maximum_angle.to('degrees')
if depression_angle > new_maximum_angle:
    print 'system is STILL unbalanced!'

#https://en.wikipedia.org/wiki/Kinetic_energy
# E=1/2*m*v^2 = F*s
# F = 1/2*m*v^2/s
body_kinectic_energy = 0.5*mass*(velocity**2)
print 'body_kinectic_energy:', body_kinectic_energy.to('joule')

