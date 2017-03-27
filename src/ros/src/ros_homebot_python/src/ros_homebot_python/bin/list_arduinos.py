#!/usr/bin/env python
"""
2015.12.19 CKS
Lists the device path for each Arduino.
"""
import sys

from ros_homebot_python import constants as c
from ros_homebot_python import utils
from ros_homebot_python.exceptions import DeviceNotFound

# BASE_DIR = os.path.dirname(os.path.realpath(__file__))
# sys.path.insert(0, os.path.join(BASE_DIR, '..'))
# sys.path.insert(0, os.path.join(BASE_DIR, '../..'))

def list_arduinos(verbose=False, target=None):
    target = (target or '').strip().lower()
    for name in sorted(c.DEVICE_SIGNATURES):
        
        if target and name.lower() != target:
            continue
            
        device = None
        try:
            device = utils.find_serial_device(name, verbose=verbose)
        except DeviceNotFound:
            pass
        
        if target:
            sys.stdout.write(device or '')
            return
        else:
            print name, device
            
    if target:
        sys.exit(1)

if __name__ == '__main__':
    
    import argparse

    parser = argparse.ArgumentParser(description='''Lists Arduinos connected on USB ports.

Examples:
    
    List all devices and their USB port:
    
        list_arduinos.py
    
    List the USB port for a specific device:
    
        list_arduinos.py torso
''', formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('target', nargs='*',
        help='The device name to look for.')
    parser.add_argument('--verbose', action='store_true', default=False)
    
    args = parser.parse_args()
    
    list_arduinos(verbose=args.verbose, target=(args.target[0] if args.target else None))
