#!../../../.env/bin/python
import os
from PIL import Image
from PIL.ImageChops import difference
import numpy as np

def only_red(im):
    """
    Strips out everything except red.
    """
    im = im.convert('RGBA')
    #im = im.convert('RGB')

    data = np.array(im)   # "data" is a height x width x 4 numpy array
    red, green, blue, alpha = data.T # Temporarily unpack the bands for readability
    #red, green, blue = data.T # Temporarily unpack the bands for readability

    # Replace all non-red areas with black
    #red_areas = red#(red < blue) & (red < green)
    #data[..., :-1][red_areas.T] = (255, 0, 0) # Transpose back needed
    #data[..., :-1][red_areas.T] = (0, 0, 0) # Transpose back needed

    im2 = Image.fromarray(red.T)
    return im2

laser_off = Image.open(os.path.expanduser('~/Desktop/laser-off.jpg'))
laser_on = Image.open(os.path.expanduser('~/Desktop/laser-on.jpg'))

laser_off_red = only_red(laser_off)
laser_off_red.save(os.path.expanduser('~/Desktop/laser-off-red.jpg'))
laser_on_red = only_red(laser_on)
laser_on_red.save(os.path.expanduser('~/Desktop/laser-on-red.jpg'))

laser_diff = difference(laser_off, laser_on)
laser_diff.save(os.path.expanduser('~/Desktop/laser-diff.jpg'))

laser_diff_red = difference(laser_off_red, laser_on_red)
laser_diff_red.save(os.path.expanduser('~/Desktop/laser-diff-red.jpg'))
