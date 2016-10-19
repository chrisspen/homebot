#!/usr/bin/env python

import libzbar as zb
from PIL import Image

im = Image.open("images/test-pure-Some text.jpg")
matches = zb.Image.from_im(im).scan()
print matches

im = Image.open("images/test-noisy1-Some text.jpg")
matches = zb.Image.from_im(im).scan()
print matches

im = Image.open("images/test-noisy2-Some text.jpg")
matches = zb.Image.from_im(im).scan()
print matches
