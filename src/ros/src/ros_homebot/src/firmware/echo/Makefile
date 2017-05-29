.RECIPEPREFIX +=
CURRENT_DIR = `pwd`
BUILD_DIR = `pwd`/.build

ARDUINO_BOARD = arduino:avr:uno
ARDUINO_DIR = /usr/share/arduino

AVRDUDE = $(ARDUINO_DIR)/hardware/tools/avr/bin/avrdude
# The board name used by Avrdude for the Uno.
AVRDUDE_PART = atmega328p
AVRDUDE_PORT = /dev/ttyACM0
# Don't change. Not dependent on the baudrate used in the sketch.
AVRDUDE_BAUDRATE = 115200

# Should be located in src.
TARGET_INO = main.ino
TARGET_HEX = $(BUILD_DIR)/$(TARGET_INO).hex

all:
    clear
    mkdir -p $(BUILD_DIR)
    arduino-builder -compile -hardware $(ARDUINO_DIR)/hardware -tools $(ARDUINO_DIR)/hardware/tools -tools $(ARDUINO_DIR)/tools-builder -libraries $(ARDUINO_DIR)/libraries -fqbn $(ARDUINO_BOARD) -build-path $(CURRENT_DIR)/.build $(CURRENT_DIR)/main.ino

clean:
    rm -Rf $(BUILD_DIR)

upload:
    ard-reset-arduino $(AVRDUDE_PORT)
    $(AVRDUDE) -C$(ARDUINO_DIR)/hardware/tools/avr/etc/avrdude.conf -v -p$(AVRDUDE_PART) -carduino -P$(AVRDUDE_PORT) -b$(AVRDUDE_BAUDRATE) -D -Uflash:w:$(TARGET_HEX):i
