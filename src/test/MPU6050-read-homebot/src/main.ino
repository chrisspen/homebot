// This example read 14 values from an MPU-6050 digital accelerometer/gyroscope,
// using the Wire library for Arduino. It sends these values over the serial line
// to a PC.
// These values are 6 bytes acceleration measurements, 2 bytes temperature
// measurement, 6 bytes gyroscope measurements.

#include "Wire.h"

#include "I2CAddresses.h"
#include "AccelGyroSensor.h"

AccelGyroSensor ag_sensor = AccelGyroSensor(ACCELGYRO_ADDR);

void setup(){
    // initiate serial communication with the PC
    Serial.begin(115200);
  
    // the following lines of code init the Wire library and wake-up the accelerometer
    Wire.begin();
    
    ag_sensor.init();
}

void loop(){
    ag_sensor.update();
    Serial.println(ag_sensor.get_reading_packet());
    //Serial.flush();
    delay(500);
}