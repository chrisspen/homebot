
Prepare the ComMotion
---------------------

1. Mount the Commotion shield onto an Arduino, and plug the Arduino into a PC.
2. Install the i2cscanner Arduino sketch.
3. Confirm that it shows the address 0x1E (decimal 30) for the Commotion. If it does not, unplug the Arduino and change the Commotion's DIP switch to set the correct address. In my experience, most Commotion boards come configured for address 0x00 by default. Flip all DIP switches so they're closest to the numbers "1 2 3 4" to use the correct address.
4. Attach a short across the RST jumperr.

Prepare the Arduino
-------------------

1. Flash the "Arduino ISP" to the Arduino.

Upload ComMotion firmware
-------------------------

1. Use "File->Upload Using Programmer" option to upload firmware to the ComMotion through the Arduino Uno. THIS IS CRITICAL. USING THE NORMAL **UPLOAD** BUTTON WILL NOT WORK.

 