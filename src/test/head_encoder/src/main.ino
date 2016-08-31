
#include "ArduinoPinout.h"

volatile int count = 0;
volatile int direction = 0; //0=unknown, +1=CW, -1=CCW 

void count_changes(){
    /*
    CW  => 11,00,11,00,...
    CCW => 10,01,10,01,...
    */
    int a = digitalRead(PAN_MOTOR_ENCODER_A);
    int b = digitalRead(PAN_MOTOR_ENCODER_B);
    direction = (a == b)? +1 : -1;
    count += 1;
}

void setup()
{
    // start serial connection
    Serial.begin(57600); // must match ino.ini
    
    pinMode(PAN_MOTOR_ENCODER_A, INPUT);
    pinMode(PAN_MOTOR_ENCODER_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(PAN_MOTOR_ENCODER_A), count_changes, CHANGE);

}

void loop()
{
    Serial.println(String("count: ") + String(count));
    Serial.println(String("direction: ") + String(direction));
    Serial.println(String("PEA: ") + String(digitalRead(PAN_MOTOR_ENCODER_A)));
    Serial.println(String("PEB: ") + String(digitalRead(PAN_MOTOR_ENCODER_B)));
    delay(500);
}
