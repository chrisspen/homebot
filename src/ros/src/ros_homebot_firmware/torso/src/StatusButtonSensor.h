#ifndef StatusButtonSensor_h
#define StatusButtonSensor_h

#include "Arduino.h"
#include "ChangeTracker.h"
#include "Sensor.h"

class StatusButtonSensor: public Sensor{

    private:
    
        // The Arduino analog pin measuring the signal
        int _pin;
        
    public:
        
        ChangeTracker<bool> value = ChangeTracker<bool>(false);

        StatusButtonSensor(){
        }
        
        StatusButtonSensor(int pin){
            _pin = pin;
            pinMode(_pin, INPUT_PULLUP);
        }
        
        virtual void update(){
            value.set(!digitalRead(_pin));
        }
        
        bool is_pressed(){
            return value.get();
        }

        virtual bool get_and_clear_changed(){
        	return value.get_and_clear_changed();
        }

        virtual String get_reading_packet(){
            return String(ID_GET_VALUE)+String(' ')+
                String(ID_STATUS_BUTTON)+String(' ')+
                String(value.get());
        }

};

#endif
