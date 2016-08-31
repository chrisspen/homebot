
#include "ArduinoPinout.h"

bool direction = true;
//int speed = 5;
int speed = 128;//mid-speed
//int speed = 255;//full speed

void setup()
{
    
    Serial.begin(57600); // must match ino.ini

    pinMode(PAN_MOTOR_PHASE, OUTPUT);
    pinMode(PAN_MOTOR_ENABLE, OUTPUT);
    pinMode(STATUS_LED_PIN, OUTPUT);
}

void loop()
{

    Serial.println("Setting PWM...");

    analogWrite(PAN_MOTOR_ENABLE, speed);
    
    Serial.println("Setting direction...");
    
    digitalWrite(STATUS_LED_PIN, direction);
    digitalWrite(PAN_MOTOR_PHASE, direction);
    direction = !direction;
    delay(3000);

}
