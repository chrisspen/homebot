# Arduino Make file.
# https://github.com/sudar/Arduino-Makefile
# https://github.com/sudar/Arduino-Makefile/blob/master/arduino-mk-vars.md
#BOARD_TAG    = uno
#BOARD_TAG    = mega
#BOARD_SUB    = atmega2560
BOARD_TAG    = uno_pro
#BOARD_TAG    = leonardo
MONITOR_PORT = /dev/ttyACM0
#MONITOR_PORT = $(shell list_arduinos.py head) #TODO:fixme
AVRDUDE_OPTS = -v
# Note, you must copy these to USER_LIB_PATH, TODO:fixme
ARDUINO_LIBS = ros_lib Servo ServoEaser
#ARDUINO_LIBS = ros_lib Servo ServoEaser SoftPWM
#ARDUINO_SKETCHBOOK = /home/$(USER)/sketchbook
ARDUINO_SKETCHBOOK = $(realpath .)/lib
USER_LIB_PATH = $(realpath .)/lib
include /usr/share/arduino/Arduino.mk
