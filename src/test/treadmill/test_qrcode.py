#!/usr/bin/env python

DATA = 'Some text'
IMAGE_FN = 'test.jpg'

# https://pypi.python.org/pypi/qrcode
# Generate code.
import qrcode
img = qrcode.make(DATA)
#print type(img)
img.save(IMAGE_FN, 'JPEG')
print 'Wrote image:', DATA

# https://pypi.python.org/pypi/qrtools/0.0.1
# sudo apt-get install libzbar-dev
# pip install qrtools
# Decode image.
import qrtools
qr = qrtools.QR()
qr.decode(IMAGE_FN)
print 'Read image:', qr.data
assert qr.data == DATA

# Decode noisy image.
qr2 = qrtools.QR()
#qr2.decode('test-noisy.jpg') # rotated, skewed, noisy, grayed
qr2.decode('test-noisy2.jpg') # rotated
print 'Read noisy image:', qr2.data
assert qr2.data == DATA, 'Invalid data: %s != %s' % (qr2.data, DATA)

#http://dsynflo.blogspot.com/2014/10/opencv-qr-code-detection-and-extraction.html
#http://wiki.ros.org/visp_auto_tracker