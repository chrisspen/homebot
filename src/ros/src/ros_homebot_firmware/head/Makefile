.RECIPEPREFIX +=
CURRENT_DIR = `pwd`
BUILD_DIR = `pwd`/.build

ARDUINO_BOARD = arduino:avr:leonardo
ARDUINO_DIR = /usr/share/arduino

AVRDUDE = $(ARDUINO_DIR)/hardware/tools/avr/bin/avrdude
# The board name used by Avrdude for the Uno.
#AVRDUDE_PART = atmega328p
# The board name used by Avrdude for the Leonardo.
AVRDUDE_PART = atmega32u4
AVRDUDE_PORT = /dev/ttyACM1
# Don't change. Not dependent on the baudrate used in the sketch.
AVRDUDE_BAUDRATE = 115200

# Should be located in src.
TARGET_INO = main.ino
TARGET_HEX = $(BUILD_DIR)/$(TARGET_INO).hex

all:
    clear
    $(shell [ ! -f src/ID.h ] && ln -s ../../common/src/ID.h src/ID.h)
    $(shell [ ! -f src/SerialPort.h ] && ln -s ../../common/src/SerialPort.h src/SerialPort.h)
    $(shell [ ! -f src/EEPROMAnything.h ] && ln -s ../../common/src/EEPROMAnything.h src/EEPROMAnything.h)
    $(shell [ ! -f src/ChangeTracker.h ] && ln -s ../../common/src/ChangeTracker.h src/ChangeTracker.h)
    $(shell [ ! -f src/StringUtils.h ] && ln -s ../../common/src/StringUtils.h src/StringUtils.h)
    $(shell [ ! -f src/Smooth.h ] && ln -s ../../common/src/Smooth.h src/Smooth.h)
    mkdir -p $(BUILD_DIR)
    arduino-builder -compile -hardware $(ARDUINO_DIR)/hardware -tools $(ARDUINO_DIR)/hardware/tools -tools $(ARDUINO_DIR)/tools-builder -libraries $(CURRENT_DIR)/lib -libraries $(ARDUINO_DIR)/libraries -fqbn $(ARDUINO_BOARD) -build-path $(CURRENT_DIR)/.build $(CURRENT_DIR)/src/main.ino

clean:
    rm -Rf $(BUILD_DIR)

upload:
    $(eval AVRDUDE_PORT := $(shell ../../ros_homebot_python/src/ros_homebot_python/bin/list_arduinos.py head))
    ard-reset-arduino $(AVRDUDE_PORT)
    $(AVRDUDE) -C$(ARDUINO_DIR)/hardware/tools/avr/etc/avrdude.conf -v -p$(AVRDUDE_PART) -carduino -P$(AVRDUDE_PORT) -b$(AVRDUDE_BAUDRATE) -D -Uflash:w:$(TARGET_HEX):i
