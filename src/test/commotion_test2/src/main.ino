
#include <Wire.h>

#include "ArduinoPinout.h"
#include "I2CAddresses.h"
#include "MotionController.h"
#include "SerialPort.h"

MotionController motion_controller;

SerialPort ser = SerialPort(57600); // must match ano.ini

void setup(){

    // Keep our power on.
    digitalWrite(POWER_OFF_PIN, LOW);
    pinMode(POWER_OFF_PIN, OUTPUT);

    ser.init();
    
    Serial.println(String("wire.begin()"));Serial.flush();
    Wire.begin(1);      // Join I2C bus as Master with address #1
    Wire.onReceive(handle_commotion_status_response);
    Wire.onRequest(handle_commotion_status_request);

    delay(1000);

    // We initialize here so we can be sure it's done after the I2C initialization.
    Serial.println(String("motor.init()"));Serial.flush();
    motion_controller = MotionController();
    motion_controller.init();
    
    Serial.println(String("stopping"));Serial.flush();
    motion_controller.stop();
    
}

void update(){

    Serial.println(String("getting update..."));
    motion_controller.update();
    Serial.println(String("update got"));
    
    Serial.println(String("status requests:")+String(requested_status_count));
    Serial.println(String("status updates:")+String(received_status_count));
    Serial.println(String("encoder0:")+String(encoders[0]));
    Serial.println(String("encoder1:")+String(encoders[1]));
    Serial.println(String("encoder2:")+String(encoders[2]));
    Serial.println(String("encoder3:")+String(encoders[3]));
    Serial.println(String("errorlog:")+String(errorlog));
    
    Serial.flush();
    
}

void loop(){

    //int speed = 1024;// full speed
    //int speed = 512;// half speed
    //int speed = 256;// quarter speed
    //int speed = 128;// 1/8 speed
    int speed = 64;// 1/16 speed

    Serial.println(String("forward"));Serial.flush();
    motion_controller.set(speed, speed);
    delay(1000);
    
    update();
    
    Serial.println(String("left"));Serial.flush();
    motion_controller.set(-speed, speed);
    delay(1000);
    
    update();
    
    Serial.println(String("backward"));Serial.flush();
    motion_controller.set(-speed, -speed);
    delay(1000);
    
    update();
    
    Serial.println(String("right"));Serial.flush();
    motion_controller.set(speed, -speed);
    delay(1000);
    
    update();

}
