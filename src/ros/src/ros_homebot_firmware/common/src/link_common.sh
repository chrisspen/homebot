#!/bin/bash
ln -s `readlink -f ChangeTracker.h` `readlink -f ../head_arduino/src`
ln -s `readlink -f ChangeTracker.h` `readlink -f ../torso_arduino/src`

ln -s `readlink -f EEPROMAnything.h` `readlink -f ../head_arduino/src`
ln -s `readlink -f EEPROMAnything.h` `readlink -f ../torso_arduino/src`

ln -s `readlink -f ArduinoTemperatureSensor.h` `readlink -f ../head_arduino/src`
ln -s `readlink -f ArduinoTemperatureSensor.h` `readlink -f ../torso_arduino/src`
