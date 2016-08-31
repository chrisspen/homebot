#!/usr/bin/env python
"""
2015-8-1 CKS
Calculates the length of the track stretched around three wheels.
"""
from math import *

def rotate(x, y, theta):
    xp = x*cos(theta) - y*sin(theta)
    yp = x*sin(theta) + y*cos(theta)
    return xp, yp

def translate(v1, v2):
    return v1[0]+v2[0], v1[1]+v2[1]

## Change these.
R = 14
BUFFER_LENGTH = 0#10
C1 = -32.5, 15.2
C2 = 32.5, 15.2
C3 = -17.5, 40.2

## Change nothing below.
D = R*2
H1 = abs(C3[1] - C1[1])
H2 = abs(C1[0] - C3[0])
H3 = abs(C3[0] - C2[0])
print 'H1:', H1
print 'H2:', H2
print 'H3:', H3

# C = sqrt(A**2 + B**2)
A = sqrt(H1**2 + H2**2)
print 'A:', A

THETA1 = tanh(H1/H2)
print 'THETA1:', THETA1

B = sqrt(H1**2 + H3**2)
print 'B:', B

THETA2 = tanh(H1/H3)
print 'THETA2:', THETA2

THETA3 = pi - THETA1 - THETA2
print 'THETA3:', THETA3

# THETA_SUM = (THETA1 + THETA2 + THETA3)
# print 'THETA_SUM:', THETA_SUM*180/pi

C = abs(C1[0] - C2[0])
print 'C:', C

P1 = translate(C1, (0, -R))
P2 = translate(C1, (0, -R))

CORNERS = 2*pi*R

print 'P1:', P1
print 'P2:', P2

TOTAL_LENGTH = A + B + C + CORNERS + BUFFER_LENGTH
print 'TOTAL_LENGTH:', TOTAL_LENGTH

TREAD_DIAMETER = TOTAL_LENGTH/pi
print 'TREAD_DIAMETER:', TREAD_DIAMETER
