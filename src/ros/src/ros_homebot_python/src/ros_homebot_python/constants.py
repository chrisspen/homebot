import os
import threading
import re

import pint

ureg = pint.UnitRegistry()

MM = ureg.millimeter
METER = ureg.meter
SEC = ureg.second
DEG = ureg.degree
RAD = ureg.radian

DRAW_LOCK = threading.RLock()

DEBOUNCE = 1

HEAD = NAME_HEAD = 'HEAD'
TORSO = NAME_TORSO = 'TORSO'
NAME_PAN = 'PAN'
NAME_TILT = 'TILT'

INDEX_HEAD = 1
INDEX_TORSO = 2

NAME_TO_INDEX = {
    NAME_HEAD: INDEX_HEAD,
    NAME_TORSO: INDEX_TORSO,
}
INDEX_TO_NAME = {
    INDEX_HEAD: NAME_HEAD,
    INDEX_TORSO: NAME_TORSO,
}

# Things to expect from running `udevadm info --query=all --name=/dev/ttyACM*`
DEVICE_SIGNATURES = {
    NAME_TORSO: [
        'ID_MODEL_FROM_DATABASE=Uno R3 (CDC ACM)',
    ],
    NAME_HEAD: [
        #'ID_MODEL=Arduino_Leonardo',
        'leonardo',
    ],
}

PALETTE = [
    ('banner', 'black', 'light gray'),
    ('streak', 'black', 'dark red'),
    ('bg', 'black', 'dark blue'),
]

HIGHLIGHT_COLOR = 'banner'

LINE_LASER_PIN = 20

ID_NULL = ''
ID_PAN_ANGLE = 'a'
ID_BUMPER = 'b'
ID_PAN_SPEED = 'c'
ID_TILT_ANGLE = 'd'
ID_EDGE = 'e'
ID_STATUS_BUTTON = 'f'
ID_BATTERY_VOLTAGE = 'g'
ID_BATTERY_TEMP = 'h'
ID_IDENTIFY = 'i'
ID_EXTERNAL_POWER = 'j'
ID_POWER_BUTTON = 'k'
ID_LED = 'l'
ID_LED_AUTO = 'm'
ID_MOTOR_SPEED = 'n'
ID_GO_TO_CENTER = 'o'
ID_PING = 'p'
ID_GET_VALUE = 'q'
ID_PAN_FULL_REV_COUNT = 'r'
ID_ALL_STOP = 's'
ID_CALIBRATE = 't'
ID_ULTRASONIC = 'u'
ID_PONG = 'v'
ID_FORCE_SENSORS = 'w'
# 'x'
ID_PAN_CENTERMARK = 'y'
ID_SET_VALUE = 'z'
ID_PAN_POWER = 'A'
ID_TILT_POWER = 'B'
ID_SONAR_POWER = 'C'
ID_ARDUINO_TEMP = 'D'
ID_IMU_EULER = 'E'
ID_RECHARGE_POWERDOWN = 'F'
ID_BATTERY_CHARGE_RATIO = 'G'
ID_IMU_ACCELEROMETER = 'H'
#ID_MICROPHONE_ENABLE = 'I'
ID_IMU_GYROSCOPE = 'J'
ID_IMU_MAGNETOMETER = 'K'
ID_LOG = 'L'
ID_MOTOR_ACCEL = 'M'
ID_IMU_CALIBRATION = 'N'
ID_MOTOR_CALIBRATION = 'O'
ID_MOTOR_ENCODER = 'P'
# 'Q'
ID_MOTOR_ERROR = 'R'
ID_GO_TO_SLEEP = 'S'
ID_SHUTDOWN = 'T'
# 'U'
# 'V'
# 'W'
ID_CRASH = 'X'
# 'Y'
ID_HASH = 'Z'

# These are used to lookup callbacks. Do Not Change.
ALL_IDS = {
    ID_PAN_ANGLE: 'pan angle',
    ID_BUMPER: 'bumper',
    ID_PAN_SPEED: 'pan speed',
    ID_TILT_ANGLE: 'tilt angle',
    ID_EDGE: 'edge',
    ID_STATUS_BUTTON: 'status button',
    ID_BATTERY_VOLTAGE: 'battery voltage',
    ID_BATTERY_TEMP: 'battery temperature',
    ID_IDENTIFY: 'identify',
    ID_EXTERNAL_POWER: 'external power',
    ID_POWER_BUTTON: 'power button',
    ID_LED: 'led',
    ID_LED_AUTO: 'led auto',
    ID_MOTOR_SPEED: 'motor speed',
    ID_MOTOR_ACCEL: 'motor acceleration',
    ID_PING: 'ping',
    ID_FORCE_SENSORS: 'force sensors',
    ID_GO_TO_CENTER: 'go to center',
    ID_GET_VALUE: 'get value',
    ID_PAN_FULL_REV_COUNT: 'pan full rev count',
    ID_CALIBRATE: 'calibrate',
    ID_ALL_STOP: 'all stop',
    ID_ULTRASONIC: 'ultrasonic',
    ID_PONG: 'pong',
    ID_PAN_CENTERMARK: 'pan centermark',
    ID_SET_VALUE: 'set value',
    ID_PAN_POWER: 'pan power',
    ID_TILT_POWER: 'tilt power',
    ID_SONAR_POWER: 'sonar power',
    ID_ARDUINO_TEMP: 'arduino temperature',
    ID_RECHARGE_POWERDOWN: 'recharge powerdown',
    ID_BATTERY_CHARGE_RATIO: 'battery charge ratio',
    ID_LOG: 'log',
    ID_GO_TO_SLEEP: 'sleep',
    ID_SHUTDOWN: 'shutdown',
    ID_CRASH: 'crash',
    ID_HASH: 'hash',
    ID_IMU_EULER: 'imu euler',
    ID_IMU_ACCELEROMETER: 'imu accelerometer',
    ID_IMU_GYROSCOPE: 'imu gyroscope',
    ID_IMU_MAGNETOMETER: 'imu magnetometer',
    ID_IMU_CALIBRATION: 'imu calibration',
    ID_MOTOR_CALIBRATION: 'motor calibration',
    ID_MOTOR_ENCODER: 'motor encoder',
    ID_MOTOR_ERROR: 'motor error',
}
NAME_TO_IDS = dict((re.sub(r'[^a-z]+', '_', v.lower()), k) for k, v in ALL_IDS.iteritems())

# These map to ROS messages.
BOTH_FORMATS_OUT = {
#     ID_ALL_STOP: [],
#     ID_IDENTIFY: [],
#     ID_LED: [('state', bool)],
#     ID_LED_AUTO: [('state', bool)],
#     ID_PING: [],
#     ID_GET_VALUE: [('id', int)],
    ID_PONG: [('total', int)],
    ID_ARDUINO_TEMP: [('temperature', float)],
    ID_MOTOR_CALIBRATION: [('name', str), ('state', int)],
}

HEAD_FORMATS_OUT = {
    ID_PAN_ANGLE: [('angle', int)],
    ID_PAN_FULL_REV_COUNT: [('count', int)],
    ID_PAN_CENTERMARK: [('state', int)],
    ID_TILT_ANGLE: [('angle', int)],
}

TORSO_FORMATS_OUT = {
    ID_BUMPER: [('index', 'uint8'), ('state', int)],
    ID_EDGE: [('index', 'uint8'), ('state', int)],
    ID_BATTERY_VOLTAGE: [('voltage', float)],
    ID_BATTERY_TEMP: [('temperature', float)],
    ID_EXTERNAL_POWER: [('state1', int), ('state2', int)],
#     ID_LED: [('state', bool)],
#     ID_LED_AUTO: [('state', bool)],
#     ID_MOTOR_SPEED: [('left', int), ('right', int)],
    ID_ULTRASONIC: [('index', 'uint8'), ('distance', int)],
#     ID_SONAR_POWER: [('state', bool)],
    ID_IMU_EULER: [('x', float), ('y', float), ('z', float)],
    ID_IMU_ACCELEROMETER: [('x', float), ('y', float), ('z', float)],
    ID_IMU_GYROSCOPE: [('x', float), ('y', float), ('z', float)],
    ID_IMU_MAGNETOMETER: [('x', float), ('y', float), ('z', float)],
    ID_IMU_CALIBRATION: [
        ('system', int),
        ('gyroscope', int),
        ('accelerometer', int),
        ('magnetometer', int),
    ],
#     ID_RECHARGE_POWERDOWN: [],
    ID_BATTERY_CHARGE_RATIO: [('charge', int)],
#     ID_GO_TO_SLEEP: [('duration', int)],
#     ID_SHUTDOWN: [],
#     ID_MOTOR_ACCEL: [('acceleration', float)],
    ID_STATUS_BUTTON: [('state', int)],
    ID_MOTOR_ENCODER: [('channel', int), ('count', int)],
    ID_MOTOR_ERROR: [('error', int)],# single byte
}

BOTH_FORMATS_IN = {
    ID_ALL_STOP: [],
    ID_LED: [('index', int), ('state', int)],
    ID_LED_AUTO: [('state', int)],
    ID_GET_VALUE: [('id', str)],
    ID_FORCE_SENSORS: [('state', int)],
}

HEAD_FORMATS_IN = {
    ID_PAN_SPEED: [('speed', 'int32')],
    ID_GO_TO_CENTER: [('type', str)],
    ID_CALIBRATE: [('type', str)],
    ID_PAN_ANGLE: [('angle', int)],
    ID_TILT_ANGLE: [('angle', int)],
    ID_TILT_POWER: [('enabled', int)],
    ID_PAN_POWER: [('enabled', int)],
#     ID_MICROPHONE_ENABLE: [('state', int)],
}

TORSO_FORMATS_IN = {
    ID_SONAR_POWER: [('enabled', int)],
    ID_MOTOR_SPEED: [('left', int), ('right', int)],
    ID_MOTOR_ACCEL: [('acceleration', int)],
    ID_RECHARGE_POWERDOWN: [],
    ID_GO_TO_SLEEP: [('duration', int)],
    ID_SHUTDOWN: [],
}

# Packets using these IDs will require acknowledgement.
ACK_IDS = set([
    ID_LED,
    ID_LED_AUTO,
    ID_SONAR_POWER,
    ID_MOTOR_SPEED,
    ID_MOTOR_ACCEL,
    ID_GO_TO_CENTER,
    ID_TILT_ANGLE,
    ID_PAN_ANGLE,
])

MOTOR_FORWARD = 'forward'
MOTOR_REVERSE = 'reverse'
MOTOR_TURN_CW = 'turn_cw'
MOTOR_TURN_CCW = 'turn_ccw'
MOTOR_BREAK = 'break'
MOTOR_PIVOT_LEFT_CW = 'pivot_left_cw'
MOTOR_PIVOT_LEFT_CCW = 'pivot_left_ccw'
MOTOR_PIVOT_RIGHT_CW = 'pivot_right_cw'
MOTOR_PIVOT_RIGHT_CCW = 'pivot_right_ccw'

# ComMotion Manual, Page 4
# The desired speed from -255 to +255. Positive values are forward, negative values are reverse.
MOTOR_MAX_SPEED = 255
MOTOR_EIGTH_SPEED = int(round(MOTOR_MAX_SPEED * 0.125))
MOTOR_QUARTER_SPEED = int(round(MOTOR_MAX_SPEED * 0.25))
MOTOR_HALF_SPEED = int(round(MOTOR_MAX_SPEED * 0.5))
MOTOR_THREE_QUARTER_SPEED = int(round(MOTOR_MAX_SPEED * 0.75))
MOTOR_DEFAULT_SPEED = MOTOR_QUARTER_SPEED
MOTOR_DEFAULT_ACCEL = 128 # velocity_units/sec
MOTOR_MIN_ACCEL = 1
MOTOR_MAX_ACCEL = MOTOR_MAX_SPEED
# Measured.
MOTOR_MAX_SPEED_REAL = 745 * MM/SEC
MOTOR_DEFAULT_ACCEL_REAL = float(MOTOR_DEFAULT_ACCEL) / MOTOR_MAX_SPEED * MOTOR_MAX_SPEED_REAL / SEC

TILT_CENTER = 90
TILT_MIN = 90-65
TILT_MAX = 90+65

PAN_MAX = 360

OK = 'OK'

PYTHON_TO_ROS_TYPES = {
    bool: 'bool',
    int: 'int32',
    float: 'float32',
    str: 'string',
}

# The maximum width of the body.
TORSO_DIAMETER_MM = 126 + 24
TORSO_DIAMETER = TORSO_DIAMETER_MM * MM

# The distance between the treads.
TORSO_TREAD_WIDTH = 100 * MM

# The distance from the ground to the center of the head.
HEIGHT_CENTER_HEIGHT_MM = 235
HEIGHT_CENTER_HEIGHT = HEIGHT_CENTER_HEIGHT_MM * MM

ARDUINO_PING_TIMEOUT = 5

MOTION_WANDER = 'wander'
MOTION_FORWARD_X_MM = 'forward_x_mm'
MOTION_TURN_X_DEGREES = 'turn_x_degrees'
MOTION_PVIOT_X_DEGREES = 'pivot_x_degrees'
MOTIONS = [
    (MOTION_WANDER, 'wander'),
    (MOTION_FORWARD_X_MM, 'forward'),
    (MOTION_TURN_X_DEGREES, 'turn'),
    (MOTION_PVIOT_X_DEGREES, 'pivot'),
]

SOUND_TTS = 'tts'
SOUND_TONE = 'tone'

# CPU temperature limits (in Celcius)
# The Pi starts to underclock itself at 85C and the components get damaged at 90C
CPU_TEMP_ERROR = 85 # over this shown error
CPU_TEMP_WARN = 82.5 # over this shown warning, below shown ok
CPU_USAGE_PERCENT_ERROR = 99
CPU_USAGE_PERCENT_WARN = 90

# CPU clock speed limits.
CPU_CLOCK_SPEED_PERCENT_ERROR = 25
CPU_CLOCK_SPEED_PERCENT_WARN = 50

# Disk limits.

DISK_USAGE_PERCENT_ERROR = 95
DISK_USAGE_PERCENT_WARN = 90

# Memory limits.

MEMORY_USAGE_PERCENT_ERROR = 95
MEMORY_USAGE_PERCENT_WARN = 90

# Links

BASE_FOOTPRINT = 'base_footprint'
BASE_LINK = 'base_link'
NECK = 'neck'
HEAD = 'head'

# Joints

FOOTPRINT_TO_TORSO_JOINT = 'footprint_to_base_link_joint'
TORSO_TO_NECK_JOINT = 'base_link_to_neck_joint'
NECK_TO_HEAD_JOINT = 'neck_to_head_joint'
HEAD_TO_CAMERA_JOINT = 'head_to_camera_joint'

# Battery limits.

BATTERY_CHARGE_RATIO_ERROR = 0.8
BATTERY_CHARGE_RATIO_WARN = 0.85

# Camera.

# http://elinux.org/Rpi_Camera_Module
CAMERA_ANGLE_OF_VIEW_H = 54
CAMERA_ANGLE_OF_VIEW_V = 41

# Diagnostic part names.

def write_ros_messages(d, prefix):
    msg_dir = '../../../ros_homebot_msgs/msg'
    for k, v in d.iteritems():
        name = re.sub(r'[^a-z]+', ' ', ALL_IDS[k])
        name = (''.join(map(str.title, name.split(' '))))
        if name != 'Pong':
            name = name + 'Change'
        #name = prefix.title() + name
        v = [('device', 'uint8')] + v
        print name, v
        with open(os.path.join(msg_dir, '%s.msg' % name), 'w') as fout:
            for _name, _type in v:
                _ros_type = PYTHON_TO_ROS_TYPES.get(_type, _type)
                print>>fout, '%s %s' % (_ros_type, _name)

def write_ros_services(d, prefix):
    msg_dir = '../../../ros_homebot_msgs/srv'
    for k, v in d.iteritems():
        name = re.sub(r'[^a-z]+', ' ', ALL_IDS[k])
        name = (''.join(map(str.title, name.split(' '))))
        #name = prefix.title() + name
        #v = [('device', 'uint8')] + v
        print name, v
        with open(os.path.join(msg_dir, '%s.srv' % name), 'w') as fout:
            for _name, _type in v:
                _ros_type = PYTHON_TO_ROS_TYPES.get(_type, _type)
                print>>fout, '%s %s' % (_ros_type, _name)
            print>>fout, '---'

def write_cpp_headers():
    # Output the IDs to a C/C++ header.
    with open('../../../ros_homebot_firmware/common/src/ID.h', 'w') as fout:
        print>>fout, '// AUTO-GENERATED. DO NOT EDIT. SEE homebot/constants.py.'
        items = [
            _ for _ in globals().items()
            if _[0].startswith('ID_')]
        for _name, _value in sorted(items, key=lambda o: o[1]):
            print>>fout, "#define %s '%s'" % (_name.ljust(4*6), _value)
        items = [
            _ for _ in globals().items()
            if _[0].startswith('NAME_') and not _[0].startswith('NAME_TO_')]
        for _name, _value in sorted(items, key=lambda o: o[0]):
            print>>fout, '#define %s "%s"' % (_name.ljust(4*6), _value)
    print 'Wrote ID.h.'

if __name__ == '__main__':
    
    write_cpp_headers()
    print '''
Now run:

    cd /home/`user`/git/homebot/src/ros
    . ./setup.bash
    catkin_make --pkg ros_homebot_msgs
'''
    
    t = HEAD_FORMATS_OUT.copy()
    t.update(BOTH_FORMATS_OUT)
    write_ros_messages(t, NAME_HEAD)
    
    t = TORSO_FORMATS_OUT.copy()
    t.update(BOTH_FORMATS_OUT)
    write_ros_messages(t, NAME_TORSO)
    
    t = HEAD_FORMATS_IN.copy()
    t.update(BOTH_FORMATS_IN)
    write_ros_services(t, NAME_HEAD)
    
    t = TORSO_FORMATS_IN.copy()
    t.update(BOTH_FORMATS_IN)
    write_ros_services(t, NAME_TORSO)
    
    os.system('cd ../../../ros_homebot_msgs; python update_makelist.py')
    
    print 'Remember to run:\n'
    print '    fab prod homebot.rebuild_messages'
