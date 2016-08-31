#ifndef StatusButtonSensor_h
#define StatusButtonSensor_h

#include "Arduino.h"

class StatusButtonSensor{

    private:
    
        // The Arduino analog pin measuring the signal
        int _pin;
        
        bool _value;
        
    public:

        StatusButtonSensor(){
        }
        
        StatusButtonSensor(int pin){
            _pin = pin;
            pinMode(_pin, INPUT_PULLUP);
        }
        
        void update(){
            _value = !digitalRead(_pin);
        }
        
        bool is_pressed(){
            return _value;
        }
        
};

#endif
