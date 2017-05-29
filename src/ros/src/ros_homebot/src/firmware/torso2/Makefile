# Arduino Make file. Refer to https://github.com/sudar/Arduino-Makefile
#BOARD_TAG    = uno
#BOARD_TAG    = mega
#BOARD_SUB    = atmega2560
BOARD_TAG    = uno_pro
MONITOR_PORT = /dev/ttyACM0
AVRDUDE_OPTS = -v
ARDUINO_LIBS = ros_lib Wire Adafruit_BNO055
ARDUINO_SKETCHBOOK = /home/$(USER)/Arduino
USER_LIB_PATH = /home/$(USER)/Arduino/libraries
#USER_LIB_PATH = $(realpath .)
#USER_LIB_PATH = ~/Arduino/libraries
#USER_LIB_PATH = /home/chris/Arduino/libraries
include /usr/share/arduino/Arduino.mk
