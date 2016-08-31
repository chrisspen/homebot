#!../../../.env/bin/python
import os
import time
import picamera

print 'Initializing...'
with picamera.PiCamera() as camera:
    camera.resolution = (1024, 768)
    print 'Starting preview..'
    camera.start_preview()
    # Camera warm-up time
    print 'Warming up...'
    time.sleep(2)
    camera.capture(os.path.expanduser('~/foo1.jpg'))
    print 'Capturing image...'
    time.sleep(2)
    camera.capture(os.path.expanduser('~/foo2.jpg'))
print 'Done.'
