
#include "ArduinoPinout.h"
#include "PanController.h"

//TODO:remove once Arduino IDE upgraded on RPi?
// Arduino Leonardo & Yun
#define digitalPinToInterrupt(p)  ( (p) == 0 ? 2 : ((p) == 1 ? 3 : ((p) == 2 ? 1 : ((p) == 3 ? 0 : ((p) == 7 ? 4 : -1)))) )

PanController pan_controller = PanController(PAN_MOTOR_ENABLE, PAN_MOTOR_PHASE, PAN_MOTOR_POSITION_REF);

void count_pan_changes(){
    /*
    CW  => 11,00,11,00,...
    CCW => 10,01,10,01,...
    */
    int a = digitalRead(PAN_MOTOR_ENCODER_A);
    int b = digitalRead(PAN_MOTOR_ENCODER_B);
    int direction = (a == b) ? +1 : -1;
    pan_controller.set_encoder_feedback(direction, 1);
    pan_controller.update();
}

long tmp_count_for_full_revolution;

void setup()
{
    
    Serial.begin(57600); // must match ino.ini

    attachInterrupt(digitalPinToInterrupt(PAN_MOTOR_ENCODER_A), count_pan_changes, CHANGE);
    
    /*
    pan_controller.begin_calibration();
    while(!pan_controller.is_calibrated()){
        pan_controller.update();
    }
    */
    EEPROM_readAnything(EE_PAN_MOTOR_FULL_REV_TICKS, tmp_count_for_full_revolution);
}

void loop()
{

    Serial.println(String("is_calibrated:")+String(pan_controller.is_calibrated()));
    Serial.println(String("centermark:")+String(pan_controller.is_centermark()));
    Serial.println(String("count_for_full_revolution:")+String(pan_controller.count_for_full_revolution.get_latest()));
    Serial.println(String("tmp_angle:")+String(pan_controller.tmp_angle));
    Serial.println(String("angle:")+String((int)round(pan_controller.actual_angle.get_latest())));
    Serial.println(String("count:")+String(pan_controller.get_count()));
    Serial.println(String("tmp_count_for_full_revolution:")+String(tmp_count_for_full_revolution));
    Serial.flush();
    delay(500);
    
}
