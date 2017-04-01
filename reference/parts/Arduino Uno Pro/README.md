Arduino Uno Pro
===============

http://www.hobbytronics.co.uk/arduino-uno-pro

https://github.com/maniacbug/mighty-1284p

Arduino UNO*Pro - Upgrade

The UNO*Pro is a simple plugin replacement for the existing ATmega328 chip (DIL socket version) and turns your existing UNO (or Duemilanove or Diecimila) into something much closer to a Mega board whilst retaining the UNO form factor and shield compatibility (see below). NOTE: Arduino main board not included.

The Arduino UNO is still the best selling Arduino board on the market, and with good reason - there are a vast range of shields available for the UNO board format.

The UNO is still a great development board, but it loses out to its big brother the Arduino Mega2560 in many areas. The Mega has 4x the flash program space, 8x as much RAM and four times as much eeprom storage space. It also has 4 serial ports and many more IO pins, but comes in a larger format and a number of shields are not compatible due to pin changes. So how can you increase the performance of the Arduino UNO? This is where the UNO*Pro upgrade board comes in.

The UNO*Pro replaces your Arduino ATmega328/168 chip with an ATmega1284 chip

Shield Compatibility
--------------------

For the most part, the UNO*Pro should be compatible with most shields designed for the Arduino UNO.

What is the same as an Arduino UNO

    All the digital pins D0 through D13
    SPI pins (SCK, MISO, MOSI)
    Analog inputs A0 - A3
    I2C pins (SDA / SCL)

What has changed

    Some of the PWM pins have changed. PWM is no longer available on D5, D6 and D11. These PWM outputs have moved to D8, D12 and D13
    Analog inputs A4 and A5 are no longer on the same pins as the I2C pins. They are now available on the extra 12 way header

NOTE: The analog-to-digital (ADC) pins seem to behave a little differently. Using digitalRead() on these pins, a higher than usual reference voltage is used, resulting in even 4V being interpreted as a logical 0. If you want to get normal behavior, you'll need to use analogRead() and then convert that 0-1024 scale to 0-1.

Installation
------------

Adding the UNO*Pro board to Arduino IDE

Before you can use your new UNO*Pro, you need to add support for the ATmega1284 chip to the Arduino IDE

Arduino 1.05 or below

If you are using Arduino 1.0.5 or below then download the Arduino ATmega1284 support files below and copy the atmega1284p directory into your arduino/hardware directory.

Arduino 1.5.5 and above

If you are using Arduino 1.5.5 or later then installation is simpler.

Under the hardware/arduino/avr/variants directory create a new directory called uno_pro

    In latest latest versions of the Arduino IDE it stores its settings in a different location
    For version 1.6.12 for example it will be something like
    C:\Users\________\AppData\Local\Arduino15\packages\arduino\hardware\avr\1.6.12\

Into this directory place the file pins_arduino.h

Next, open the hardware\arduino\avr\boards.txt file with a text editor and add the following entries for the UNO*Pro board

##############################################################

uno_pro.name=Arduino Uno*Pro

uno_pro.upload.tool=avrdude
uno_pro.upload.protocol=arduino
uno_pro.upload.maximum_size=130048
uno_pro.upload.maximum_data_size=16384
uno_pro.upload.speed=115200

uno_pro.bootloader.tool=avrdude
uno_pro.bootloader.low_fuses=0xFF
uno_pro.bootloader.high_fuses=0xDE
uno_pro.bootloader.extended_fuses=0xFD
uno_pro.bootloader.unlock_bits=0x3F
uno_pro.bootloader.lock_bits=0x0F
uno_pro.bootloader.file=optiboot/optiboot_atmega1284p.hex

uno_pro.build.mcu=atmega1284p
uno_pro.build.f_cpu=16000000L
uno_pro.build.board=AVR_UNO_PRO
uno_pro.build.core=arduino
uno_pro.build.variant=uno_pro

##############################################################
Programming the new UNO*Pro

Programming your upgraded UNO*Pro board is just the same as before using the normal Arduino IDE software. The only difference is that you need to select Arduino UNO*Pro rather than Arduino Uno from the Boards menu.

Start up the Arduino IDE and select Arduino UNO*Pro.