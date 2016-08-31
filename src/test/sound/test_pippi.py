#!/usr/bin/env python
"""
pip install pippi
"""
import sys
from pippi import dsp
out = dsp.tone(dsp.stf(5), freq=220, amp=0.2)
out = dsp.env(out, 'hann')
#dsp.write(out, 'hello')
#'hello.wav'
dsp.write(sys.stdout)
