#!../../../.env/bin/python


import numpy as np
from PIL import Image
from PIL.ImageChops import difference

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

def normalize(arr):
    """
    Linear normalization
    http://en.wikipedia.org/wiki/Normalization_%28image_processing%29
    """
    arr = arr.astype('float')
    # Do not touch the alpha channel
    for i in range(3):
        minval = arr[...,i].min()
        maxval = arr[...,i].max()
        if minval != maxval:
            arr[...,i] -= minval
            arr[...,i] *= (255.0/(maxval-minval))
    return arr

def find_line(img, out_fn, filter_outliers=True):
    width, height = size = img.size
    max_height = height
    x = img.convert('L')
    out = Image.new("L", x.size, "black")
    pix = out.load()
    y = np.asarray(x.getdata(), dtype=np.float64).reshape((x.size[1], x.size[0]))
    #print y.shape

    # Find row in each column with maximum brightness.
    i = 0
    laser_measurements = [0]*width # [row]
    laser_brightness = [0]*width
    for col_i in xrange(y.shape[1]):
        i += 1
        col_max = max([(y[row_i][col_i], row_i) for row_i in xrange(y.shape[0])])
        col_max_brightness, col_max_row = col_max
        #print col_i, col_max
        #pix[col_i, col_max_row] = 255
        laser_measurements[col_i] = col_max_row
        laser_brightness[col_i] = col_max_brightness

    # Ignore all columns with dim brightness outliers.
    # These usually indicate a region where the laser is obsorbed or otherwise scattered too much to see.
    brightness_std = np.std(laser_brightness)
    brightness_mean = np.mean(laser_brightness)
#     print 'brightness_std:',brightness_std
#     print 'brightness_mean:',brightness_mean
    for col_i, col_max_row in enumerate(laser_measurements):
        if not filter_outliers or laser_brightness[col_i] > brightness_mean-brightness_std:
            if col_max_row >= max_height/2:
                # Assuming the laser is mounted below the camera,
                # we can assume all points above the centerline are noise.
                pix[col_i, col_max_row] = 255
    out.save(out_fn)

def demo_normalize():

    img = Image.open('data/laser-off.jpg').convert('RGBA')
    img01 = img
    arr = np.array(img)
    new_img = Image.fromarray(normalize(arr).astype('uint8'),'RGBA')
    new_img = only_red(new_img)
    new_img.save('data/laser-off-normalized.jpg')
    new_img1 = new_img

    img = Image.open('data/laser-on.jpg').convert('RGBA')
    img02 = img
    arr = np.array(img)
    new_img = Image.fromarray(normalize(arr).astype('uint8'),'RGBA')
    new_img = only_red(new_img)
    new_img.save('data/laser-on-normalized.jpg')
    new_img2 = new_img

    laser_diff0 = difference(img01, img02)
    laser_diff0.save('data/laser-diff0.jpg')

    laser_diff1 = difference(new_img1, new_img2)
    laser_diff1.save('data/laser-diff1.jpg')

    find_line(laser_diff0, 'data/laser-line0.jpg', filter_outliers=0)
    find_line(laser_diff1, 'data/laser-line1.jpg')

demo_normalize()
