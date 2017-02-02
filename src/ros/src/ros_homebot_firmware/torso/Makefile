.RECIPEPREFIX +=
CURRENT_DIR = `pwd`
BUILD_DIR = `pwd`/.build

ARDUINO_BOARD = arduino:avr:uno
ARDUINO_DIR = /usr/share/arduino

AVRDUDE = $(ARDUINO_DIR)/hardware/tools/avr/bin/avrdude
# The board name used by Avrdude for the Uno.
AVRDUDE_PART = atmega328p
AVRDUDE_PORT = /dev/ttyACM0
# Should match main.ino and ros_homebot_python/nodes/torso_node.py
# 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, or 115200
AVRDUDE_BAUDRATE = 115200

# Should be located in src.
TARGET_INO = main.ino
TARGET_HEX = $(BUILD_DIR)/$(TARGET_INO).hex

all:
    clear
    $(shell [ ! -f src/ID.h ] && ln -s ../../common/src/ID.h src/ID.h)
    $(shell [ ! -f src/SerialPort.h ] && ln -s ../../common/src/SerialPort.h src/SerialPort.h)
    $(shell [ ! -f src/ArduinoTemperatureSensor.h ] && ln -s ../../common/src/ArduinoTemperatureSensor.h src/ArduinoTemperatureSensor.h)
    $(shell [ ! -f src/ChangeTracker.h ] && ln -s ../../common/src/ChangeTracker.h src/ChangeTracker.h)
    $(shell [ ! -f src/EEPROMAnything.h ] && ln -s ../../common/src/EEPROMAnything.h src/EEPROMAnything.h)
    $(shell [ ! -f src/StringUtils.h ] && ln -s ../../common/src/StringUtils.h src/StringUtils.h)
    mkdir -p $(BUILD_DIR)
    arduino-builder -compile -hardware $(ARDUINO_DIR)/hardware -tools $(ARDUINO_DIR)/hardware/tools -tools $(ARDUINO_DIR)/tools-builder -libraries $(CURRENT_DIR)/lib -libraries $(ARDUINO_DIR)/libraries -fqbn $(ARDUINO_BOARD) -build-path $(CURRENT_DIR)/.build $(CURRENT_DIR)/src/main.ino

clean:
    rm -Rf $(BUILD_DIR)

upload:
    $(eval AVRDUDE_PORT := $(shell ../../ros_homebot_python/src/ros_homebot_python/bin/list_arduinos.py torso))
    ard-reset-arduino $(AVRDUDE_PORT)
    $(AVRDUDE) -C$(ARDUINO_DIR)/hardware/tools/avr/etc/avrdude.conf -v -p$(AVRDUDE_PART) -carduino -P$(AVRDUDE_PORT) -b$(AVRDUDE_BAUDRATE) -D -Uflash:w:$(TARGET_HEX):i
