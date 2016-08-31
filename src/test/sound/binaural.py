"""
pip install -e git://github.com/zacharydenton/wavebender.git#egg=wavebender

python binaural.py | aplay

DOES NOT WORK
"""

#!/usr/bin/env python
from wavebender import *

channels = ((sine_wave(170.0, amplitude=0.1),),
            (sine_wave(178.0, amplitude=0.1),))

samples = compute_samples(channels)
write_wavefile(stdout, samples)