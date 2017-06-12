#!../../../.env/bin/python
"""
https://shaneormonde.wordpress.com/2014/01/25/webcam-laser-rangefinder/

theta = arctan(h/actual_d)

theta = pfc * rpc + ro
"""
import sys
from math import *
from scipy import stats
import numpy as np

from laser_range_finder import pixels_to_distance

def percent_error(expected, actual):
    return (expected - actual)/float(actual)*100

pixel_readings = [98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 98, 99, 99, 99, 99, 98, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 101, 101, 101, 101, 101, 101, 101, 101, 101, 101, 101, 101, 101, 101, 101, 101, 101, 101, 102, 102, 102, 102, 102, 102, 102, 102, 102, 102, 102, 102, 102, 102, 102, 102, 102, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 83, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 80, 80, 80, 80, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 80, 80, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 81, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 82, 83, 83, 83, 83, 83, 83, 83, 83, 84, 84, 84, 84, 84, 84, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 108, 116]

#calibrate-laser-on-20151101.jpg

distances = dict(
    a=350,#mm
    b=400,
    c=180,
    d=650,
    e=650,
    f=2450,
)

positions = dict(
    #reference_point: column,
    a=14,
    b=66,
    c=179,
    d=249,
    e=313,
    f=319,
)

h = 22.5 # Distance between laser and camera in mm.

image_width = 320
image_height = 240

measurements = []
for letter in sorted(positions.keys()):
    actual_d = distances[letter]
    pix_dist = pixel_readings[positions[letter]]
    assert pix_dist > 0
    pfc = abs(pix_dist - image_height/2)
    #theta = atan(h/actual_d)
    measurements.append((actual_d, pfc))

# reference settings
# theta = .02960246
# pix_dist = 13
# actual_d = 180
# #theta = atan(h/actual_d)
# h = tan(theta)*actual_d
# # print 'h:', h
# measurements = [
#     #(actual_d,pix_dist),
#     (180,13),
#     (160,16),
#     (140,20),
#     (120,25),
#     (100,33),
#     (80,44),
#     (60,63),
#     (40,103),
# ]

print '\nmeasurements:', measurements

x = [_pix_dist for _actual_d, _pix_dist in measurements]
y = [atan(h/_actual_d) for _actual_d, _pix_dist in measurements]

slope, intercept, r_value, p_value, std_err = stats.linregress(x, y)
print '\nlinreg:', slope, intercept, r_value, p_value, std_err
# y = m * x + b => theta = rpc * pfc + ro
# slope = m = rpc
# intercept = b = ro
rpc = slope
ro = intercept
print '\n--rpc=%s --ro=%s' % (slope, intercept)

estimated_distances = pixels_to_distance(
    pixel_rows=pixel_readings,
    rpc=rpc,
    ro=ro,
    h=h,
    max_height=image_height,
    max_width=image_width,
)
print '\nestimated_distances:', estimated_distances
estimated_distances2 = [_v for _i, _v in enumerate(estimated_distances) if _i in positions.values()]
print '\nestimated_distances:', estimated_distances2


print '\npixels from center,calc D (mm),actual D (mm),% error'
differences = []
for letter in positions.keys():
    actual_d = distances[letter]
    pix_dist = pixel_readings[positions[letter]]
    pfc = abs(pix_dist - image_height/2)
    #pfc0 = abs(pix_dist - image_height/2)
    #theta = atan(h/actual_d)
    estimated_d = estimated_distances[positions[letter]]
    print '%s,%s,%s,%s' % (pfc, estimated_d, actual_d, percent_error(estimated_d, actual_d))
    differences.append(abs(actual_d - estimated_d))
print '\naverage error:', sum(differences)/float(len(differences))


