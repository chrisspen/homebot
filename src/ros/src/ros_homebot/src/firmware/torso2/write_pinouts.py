#!/usr/bin/env python
"""
Writes the ArduinoPinout.h files.

Usage:

    write_pinout.py head
    write_pinout.py torso
    
"""
from __future__ import print_function
import os, sys, csv

#device = sys.argv[1]

fn = 'ArduinoUnoProPinout.csv'
fout = open('ArduinoPinout.h' % device, 'w')

print('''/*
AUTO-GENERATED DO NOT MODIFY DIRECTLY

To modify, edit {fn} and then run:

    write_pinouts.py

Pinouts for the Arduino located in the {device}.
*/'''.format(device=device, fn=fn), file=fout)
reader = csv.DictReader(open(fn, 'r'))
for line in reader:
    name = line['Variable'].strip()
    if not name:
        continue
    label = line['Label'].strip()
    pin = line['Arduino Pin'].strip()
    if pin.startswith('D'):
        pin = pin[1:]
    print('#define %s %s // %s' % (name.ljust(30), pin, label), file=fout)
