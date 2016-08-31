
#include "ArduinoPinout.h"
#include "SerialPort.h"

SerialPort ser = SerialPort(57600);

void setup()
{
    
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);
    
    pinMode(PAN_MOTOR_PHASE, OUTPUT);
    pinMode(PAN_MOTOR_ENABLE, OUTPUT);

}

void loop()
{

    Packet p = ser.read();
    
    if(p.id == ID_IDENTIFY){
        ser.write(String(ID_IDENTIFY)+String(' ')+String("HEAD"));
    }else if(p.id == ID_LED){
        if(p.data == "0"){
            digitalWrite(STATUS_LED_PIN, LOW);
        }else if(p.data == "1"){
            digitalWrite(STATUS_LED_PIN, HIGH);
        }else if(p.data == "toggle"){
            digitalWrite(STATUS_LED_PIN, HIGH - digitalRead(STATUS_LED_PIN));
        }
        ser.write(String(ID_LED)+String(' ')+String(digitalRead(STATUS_LED_PIN)));
    }else if(p.id == ID_PING){
        ser.write(String("PONG"));
    }else if(p.id == ID_PAN_ANGLE){
        if(p.data.length()
    }

}
