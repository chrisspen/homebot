
#include <Servo.h>

#include "TiltController.h"

#define SERVO_SET_PIN 6
//#define SERVO_GET_PIN A0 //doesn't work? causes servo control pin to spasm?
#define SERVO_GET_PIN A1

TiltController tc = TiltController(SERVO_SET_PIN, SERVO_GET_PIN);

void setup()
{
    Serial.begin(57600); // must match ino.ini
    
    //tc.calibrate();
    tc.start();
}

void loop()
{

    int deg_inc = 1;

    //tc.get_servo().write(90); delay(500); return;//TODO:remove

    Serial.println("Servo moving up...");
    for(int target_degrees = tc.get_lower_endstop_degrees(); target_degrees < tc.get_upper_endstop_degrees(); target_degrees += deg_inc)
    {
        Serial.println(String("Setting servo upper pos:")+String(target_degrees));Serial.flush();
        tc.set_target_degrees(target_degrees);
        tc.update();
        Serial.println(tc.get_info());Serial.flush();
        tc.wait_for_target();
        delay(50);
    }
    
    Serial.println("Servo moving down...");
    for(int target_degrees = tc.get_upper_endstop_degrees(); target_degrees > tc.get_lower_endstop_degrees(); target_degrees -= deg_inc)
    {
        Serial.println(String("Setting servo lower pos:")+String(target_degrees));Serial.flush();
        tc.set_target_degrees(target_degrees);
        tc.update();
        Serial.println(tc.get_info());Serial.flush();
        tc.wait_for_target();
        delay(50);
    }

    /*
    int actual_pos = analogRead(SERVO_GET_PIN);
    Serial.println(String("actual_pos:")+String(actual_pos));
    Serial.flush();
    delay(500);
    */

}
