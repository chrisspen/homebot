#!/usr/bin/env python
from __future__ import print_function

import pint

ureg = pint.UnitRegistry()

mm = ureg.millimeter
sec = ureg.second

speed = 186. * mm/sec
sensor_latency = .5 * sec

min_stop_distance = speed * sensor_latency

print('min_stop_distance:', min_stop_distance)
