#!../../../.env/bin/python
import os
import sys
import time
import atexit
import subprocess
import grp
import io
from math import pi, tan
import argparse

from PIL import Image
from PIL.ImageChops import difference
import numpy as np
import wiringpi2
import picamera

from laser_range_finder import LaserRangeFinder, utils

LASER_EN_PIN = 18 # GPIO 18, (P1 pin 12)

#JPEG_QUALITY = 100
JPEG_QUALITY = 80

_exported_pins = set()

def export_pin(pin):
    try:
        _exported_pins.add(str(pin))
        with open("/sys/class/gpio/export", "w") as fout:
            fout.write(str(pin))
            fout.flush()

        #TODO:fix? this fixes the PermissionDenied errors, but GPIO writes are still ignored
        # Wait for the new gpio folder to be created.
        # This is not instantaneous.
        target = "/sys/class/gpio/gpio{pin}/direction".format(pin=pin)
        while not os.path.isfile(target):
            time.sleep(.1)
        while 1:
            target_info = os.stat(target)
            target_group = grp.getgrgid(target_info.st_gid)[0]
            if target_group == 'gpio':
                break
            time.sleep(.1)

        target = "/sys/class/gpio/gpio{pin}/value".format(pin=pin)
        while not os.path.isfile(target):
            time.sleep(.1)
        while 1:
            target_info = os.stat(target)
            target_group = grp.getgrgid(target_info.st_gid)[0]
            if target_group == 'gpio':
                break
            time.sleep(.1)

    except IOError:
        print "INFO: GPIO %s already Exists, skipping export" % pin

def unexport_pin(pin):
    try:
        with open("/sys/class/gpio/unexport", "w") as fout:
            fout.write(str(pin))
            fout.flush()
    except IOError:
        print "INFO: GPIO %s already removed, skipping unexport" % pin

def unexport_pins():
    for pin in list(_exported_pins):
        unexport_pin(pin)
        _exported_pins.remove(pin)

def pinMode(pin, mode):

    # Enable the pin for setting as input/output.
    export_pin(pin)
    assert os.path.isfile("/sys/class/gpio/gpio{pin}/direction".format(pin=pin))

    # Set pin usage.
    #TODO:fix? this has to be run twice in separate processes to take effect
    #even if the second pass suceeds, the GPIO line won't be accessible until the next script's run
    max_retries = 10
    for retry in xrange(max_retries):
#         print 'retry:', retry
        try:
            mode_name = 'out' if mode else 'in'
            with open("/sys/class/gpio/gpio{pin}/direction".format(pin=pin), "w") as fout:
                fout.write(mode_name)
                fout.flush()
#             print 'success'
            break
        except IOError as e:
            if 'Permission denied' in str(e) and retry+1 < max_retries:
                print e
                time.sleep(.1)
            else:
                raise

# def only_red(im):
#     """
#     Strips out everything except red.
#     """
#     im = im.convert('RGBA')
#     #im = im.convert('RGB')
#
#     data = np.array(im)   # "data" is a height x width x 4 numpy array
#     red, green, blue, alpha = data.T # Temporarily unpack the bands for readability
#     #red, green, blue = data.T # Temporarily unpack the bands for readability
#
#     # Replace all non-red areas with black
#     #red_areas = red#(red < blue) & (red < green)
#     #data[..., :-1][red_areas.T] = (255, 0, 0) # Transpose back needed
#     #data[..., :-1][red_areas.T] = (0, 0, 0) # Transpose back needed
#
#     im2 = Image.fromarray(red.T)
#     return im2

# def calculate_distance(laser_off_img, laser_on_img, vert_fov_deg=41.41, ro=-0.21, h=22.5, save=False):
#     """
#     Returns a list of distances in millimeters given a list of rows where a horizontal laser line was detected.
#     """
#
#     # Find laser line difference.
#     if isinstance(laser_off_img, basestring):
#         laser_off_img = Image.open(laser_off_img)
#     if isinstance(laser_on_img, basestring):
#         laser_on_img = Image.open(laser_on_img)
#     laser_off_red_img = only_red(laser_off_img)
#     #laser_off_red.save(os.path.expanduser('~/Desktop/laser-off-red.jpg'))
#     laser_on_red_img = only_red(laser_on_img)
#     #laser_on_red.save(os.path.expanduser('~/Desktop/laser-on-red.jpg'))
#     laser_diff_red_img = difference(laser_off_red_img, laser_on_red_img)
#     #laser_diff_red_img.save(os.path.expanduser('~/Desktop/laser-diff-red.jpg'))
#
#     # Estimate the pixels that are the laser.
#     width, height = size = laser_diff_red_img.size
#     max_height = height
#     x = laser_diff_red_img.convert('L')
#     if save:
#         out = Image.new("L", x.size, "black")
#         pix = out.load()
#     y = np.asarray(x.getdata(), dtype=np.float64).reshape((x.size[1], x.size[0]))
#     #print y.shape
#     i = 0
#     laser_measurements = [0]*width # [row]
#     for col_i in xrange(y.shape[1]):
#         i += 1
#         col_max = max([(y[row_i][col_i], row_i) for row_i in xrange(y.shape[0])])
#         col_max_brightness, col_max_row = col_max
#         #print col_i, col_max
#         if save:
#             pix[col_i, col_max_row] = 255
#         laser_measurements[col_i] = col_max_row
#     #print i
#     if save:
#         out.save(os.path.expanduser('~/laser-line.jpg'))
#     #print laser_measurements
#
#     #https://sites.google.com/site/todddanko/home/webcam_laser_ranger
#     #https://shaneormonde.wordpress.com/2014/01/25/webcam-laser-rangefinder/
#     #https://www.raspberrypi.org/documentation/hardware/camera.md
#     #Horizontal field of view     53.50 +/- 0.13 degrees
#     #Vertical field of view     41.41 +/- 0.11 degress
#     # h = distance between laser and camera in mm
#
#     # D = h/tan(theta)
#     # theta = pfc*rpc + ro
#
#     # pfc = ? # number of pixels from center of focal plane
#     # Calculated per pixel.
#
#     # rpc = ? # radians per pixel pitch
#     # 180 deg=pi rad
#     rpc = (vert_fov_deg*pi/180.)/max_height
#
#     # ro = ? # radian offset (compensates for alignment errors)
#
#     D_lst = []
#     for laser_row_i in laser_measurements:
#         pfc = abs(laser_row_i - max_height)
#         D = h/tan(pfc*rpc + ro)
#         D_lst.append(D)
#
#     return D_lst

# def compress_list(lst, bins=10):
#     """
#     Averages a list of numbers into smaller bins.
#     """
#     new_lst = []
#     chunk_size = int(round(len(lst)/float(bins)))
#     for bin in xrange(bins):
#         samples = lst[bin*chunk_size:bin*chunk_size+chunk_size]
#         new_lst.append(sum(samples)/float(len(samples)))
#     return new_lst

def cleanup():
    global camera
    wiringpi2.digitalWrite(LASER_EN_PIN, 0)
    #TODO:re-enable? disabled for now so we don't lose our pin configuration
#     unexport_pins()
    if camera:
        camera.close()

atexit.register(cleanup)

#capture_delay = .2 # seconds
capture_delay = 0 # seconds
camera = None

TOP = 'top'
BOTTOM = 'bottom'
LASER_POSITIONS = (TOP, BOTTOM)

def run(continuous=False, verbose=False, save=False, laser_position=None, as_pfc=False, rpc=None, ro=None):
    global camera

    assert laser_position in LASER_POSITIONS

    def log(*args):
        if verbose:
            return ' '.join(map(str, args))

    log('Initializing GPIO...')
    wiringpi2.wiringPiSetupSys()
    # Set laser control pin to output.
    #wiringpi2.pinMode(LASER_EN_PIN, 1)#TODO:has no effect?!
    pinMode(LASER_EN_PIN, 1)
    log('GPIO initialized.')

#     def process():
#         while 1:
#             t00 = time.clock()
#
#             log('Recording reference image with laser off...')
#             #time.sleep(capture_delay)
#             stream = io.BytesIO()
#             yield stream
# #             camera.capture(
# #                 #os.path.expanduser('~/laser-off.jpg'),
# #                 stream,
# #                 format='jpeg',
# #                 quality=JPEG_QUALITY,
# #                 use_video_port=True,
# #             )
#             stream.seek(0)
#             laser_off_image = Image.open(stream)
#             if save:
#                 laser_off_image.save(os.path.expanduser('~/laser-off.jpg'))
#
#             log('Turning laser on...')
#             wiringpi2.digitalWrite(LASER_EN_PIN, 1)
#
#             log('Recording reference image with laser on...')
#             #time.sleep(capture_delay)
#             stream = io.BytesIO()
#             yield stream
# #             camera.capture(
# #                 #os.path.expanduser('~/laser-on.jpg'),
# #                 stream,
# #                 format='jpeg',
# #                 quality=JPEG_QUALITY,
# #                 use_video_port=True,
# #             )
#             stream.seek(0)
#             laser_on_image = Image.open(stream)
#             if save:
#                 laser_on_image.save(os.path.expanduser('~/laser-on.jpg'))
#
#             log('Turning laser off...')
#             wiringpi2.digitalWrite(LASER_EN_PIN, 0)
#
#             log('Calculating distance...')
#             t0 = time.clock()
#             distance = calculate_distance(
# #                 os.path.expanduser('~/laser-off.jpg'),
# #                 os.path.expanduser('~/laser-on.jpg'),
#                 laser_off_image,
#                 laser_on_image,
#                 save=save,
#             )
#             td = time.clock() - t0
#             distance = map(int, compress_list(distance, bins=10))
#             print 'distance calculation took %s seconds' % td
#             print 'distance:', distance
#
#             tdd = time.clock() - t00
#             print 'fps:', 1./tdd
#             print 'exposure_speed:', camera.exposure_speed
#             print 'shutter_speed:', camera.shutter_speed
#             print 'camera.framerate:', camera.framerate
#
#             if not continuous:
#                 break

    distance = None
    with picamera.PiCamera() as camera:

        log('Initializing camera...')
        #camera.resolution = (1024, 768)
        camera.resolution = (320, 240)
        if laser_position == BOTTOM:
            camera.rotation = 180
        camera.meter_mode = 'backlit'
        #camera.start_preview()
        #camera.led = False
        #camera.shutter_speed = 30000 # microseconds (default)
        camera.shutter_speed = 10000 # microseconds
        #camera.shutter_speed = 5000 # microseconds
        #camera.shutter_speed = 1000 # microseconds
        camera.framerate = 80
        log('Camera initialized.')

        #TODO:fix? ignores KeyboardInterrupt?
#         camera.capture_sequence(process(), format='jpeg', use_video_port=True)

        if rpc is not None:
            rpc = float(rpc)

        if ro is not None:
            ro = float(ro)

        while 1:
            t00 = time.clock()

            log('Recording reference image with laser off...')
            #time.sleep(capture_delay)
            # Take at least two images to ensure proper exposure.
            for _i in xrange(2):
                stream = io.BytesIO()
                camera.capture(
                    #os.path.expanduser('~/laser-off.jpg'),
                    stream,
                    format='jpeg',
                    quality=JPEG_QUALITY,
                    use_video_port=True,
                )
                stream.seek(0)
                laser_off_image = Image.open(stream)
                if save:
                    laser_off_image.save(os.path.expanduser('~/laser-off%i.jpg' % _i))

            log('Turning laser on...')
            wiringpi2.digitalWrite(LASER_EN_PIN, 1)

            log('Recording reference image with laser on...')
            #time.sleep(capture_delay)
            stream = io.BytesIO()
            camera.capture(
                #os.path.expanduser('~/laser-on.jpg'),
                stream,
                format='jpeg',
                quality=JPEG_QUALITY,
                use_video_port=True,
            )
            stream.seek(0)
            laser_on_image = Image.open(stream)
            if save:
                laser_on_image.save(os.path.expanduser('~/laser-on.jpg'))

            log('Turning laser off...')
            wiringpi2.digitalWrite(LASER_EN_PIN, 0)

            try:
                log('Calculating distance...')
                t0 = time.clock()
                lrf = LaserRangeFinder(
                    laser_position=laser_position,
                    rpc=rpc or 0.00103721750257,
                    ro=ro or 0.418921972295,
                )
                distances = lrf.get_distance(
                    off_img=laser_off_image,#os.path.join(CURRENT_DIR, '../../docs/images/sample1/sample-a-0.jpg'),
                    on_img=laser_on_image,#os.path.join(CURRENT_DIR, '../../docs/images/sample1/sample-b-0.jpg'),
                    save_images_dir='~',
                    as_pfc=as_pfc,
                )
#                 distance = calculate_distance(
#     #                 os.path.expanduser('~/laser-off.jpg'),
#     #                 os.path.expanduser('~/laser-on.jpg'),
#                     laser_off_image,
#                     laser_on_image,
#                     save=save,
#                 )
                td = time.clock() - t0
                #distance = map(int, compress_list(distance, bins=10))
                print 'distance calculation took %s seconds' % td
                print 'distances:', distances
                distances_10 = utils.compress_list(distances, bins=10, as_int=True)
                print 'distances_10:', distances_10

                tdd = time.clock() - t00
                print 'fps:', 1./tdd
                print 'exposure_speed:', camera.exposure_speed
                print 'shutter_speed:', camera.shutter_speed
                print 'camera.framerate:', camera.framerate
            except IOError as e:
                # On rare occassions we see:
                # IOError: broken data stream when reading image file
                print>>sys.stderr, e

            if not continuous:
                break

    return distance

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Measure distance using laser.')
    parser.add_argument('--continuous', action='store_true', default=False,
        help='Continually outputs distance readings.')
    parser.add_argument('--verbose', action='store_true', default=False,
        help='Outputs debugging messages.')
    parser.add_argument('--save', action='store_true', default=False,
        help='Saves images to ~.')
    parser.add_argument('--as-pfc', action='store_true', default=False,
        help='Return measurements as pixels-from-center.')
    parser.add_argument('--laser-position', default='bottom',
        help='Position of the laser relative to the camera. '
        'Assumes the camera\'s default "up" is positioned with the laser below. '
        'top|bottom')
    parser.add_argument('--rpc', default=0.00329743488774,
        help='Radians per pixel pitch.')
    parser.add_argument('--ro', default=-0.00494930380293,
        help='Radian offset.')
    args = parser.parse_args()

    run(**args.__dict__)
