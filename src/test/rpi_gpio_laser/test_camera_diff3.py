#!../../../.env/bin/python
#http://stackoverflow.com/questions/9111711/get-coordinates-of-local-maxima-in-2d-array-above-certain-value
import os
from PIL import Image, ImageOps
import numpy as np
import math
from math import pi, tan

#http://stackoverflow.com/questions/23935840/converting-an-rgb-image-to-grayscale-and-manipulating-the-pixel-data-in-python
x = Image.open(os.path.expanduser('~/Desktop/laser-diff-red.jpg'), 'r')
width, height = size = x.size
x = x.convert('L')

out = Image.new("L", x.size, "black")
pix = out.load()

y = np.asarray(x.getdata(), dtype=np.float64).reshape((x.size[1], x.size[0]))
#print y.shape
i = 0
laser_measurements = [0]*width # [row]
for col_i in xrange(y.shape[1]):
    i += 1
    col_max = max([(y[row_i][col_i], row_i) for row_i in xrange(y.shape[0])])
    col_max_brightness, col_max_row = col_max
    print col_i, col_max
    pix[col_i, col_max_row] = 255
    laser_measurements[col_i] = col_max_row
#print i
out.save(os.path.expanduser('~/Desktop/laser-line.jpg'))
print laser_measurements

def calculate_distance(measurements, max_height, vert_fov_deg=41.41, ro=-0.21):
    """
    Returns a list of distances in millimeters given a list of rows where a horizontal laser line was detected.
    """
    #https://sites.google.com/site/todddanko/home/webcam_laser_ranger
    #https://shaneormonde.wordpress.com/2014/01/25/webcam-laser-rangefinder/
    #https://www.raspberrypi.org/documentation/hardware/camera.md
    #Horizontal field of view     53.50 +/- 0.13 degrees
    #Vertical field of view     41.41 +/- 0.11 degress
    h = 22.5 # distance between laser and camera in mm

    # D = h/tan(theta)
    # theta = pfc*rpc + ro

    # pfc = ? # number of pixels from center of focal plane
    # Calculated per pixel.

    # rpc = ? # radians per pixel pitch
    # 180 deg=pi rad
    rpc = (vert_fov_deg*pi/180.)/max_height

    # ro = ? # radian offset (compensates for alignment errors)

    D_lst = []
    for laser_row_i in measurements:
        pfc = abs(laser_row_i - max_height)
        D = h/tan(pfc*rpc + ro)
        D_lst.append(D)

    return D_lst

def compress_list(lst, bins=10):
    new_lst = []
    chunk_size = int(round(len(lst)/float(bins)))
    for bin in xrange(bins):
        samples = lst[bin*chunk_size:bin*chunk_size+chunk_size]
        new_lst.append(sum(samples)/float(len(samples)))
    return new_lst

distances = calculate_distance(laser_measurements, max_height=height)
print 'distances:'
for _ in distances:
    print _

distances_10 = map(int, compress_list(distances, bins=10))
print 'distances_10:', distances_10
