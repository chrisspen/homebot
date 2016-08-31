#!/usr/bin/env python
"""
2015.2.22 CKS
Tools for helping calculate sizes so they're not improperly rasterized
by the printer.

e.g. Given a 0.4mm nozzle, you can't print something that's exactly 1mm
but you can print 1.2mm or 0.8mm.
"""
import sys
from math import floor
from decimal import Decimal

def fix_size(diameter, nozzle=0.4):
    """
    Returns the closest size capable of being rendered by the given nozzle.
    """
    _diameter = Decimal(str(diameter))
    _nozzle = Decimal(str(nozzle))
    return floor(_diameter/_nozzle)*nozzle
    
print fix_size(float(sys.argv[1]))
