/*
 * Copyright 2017 Chris Spencer
 * 
 * Simple program that immediately writes everything received via serial.
 * 
 * For use testing the serial communication.
 */

void setup() {
    Serial.begin(57600);
    while (!Serial) {
        // Wait for serial port to connect.
        // Needed for native USB port only
        delay(1);
    }
}

void loop() {
    if (Serial.available()) {
        Serial.write(Serial.read());
    }
}
