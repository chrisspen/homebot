#ifndef BooleanSensor_h
#define BooleanSensor_h

#include "Arduino.h"

#include "ID.h"
#include "ChangeTracker.h"
#include "Sensor.h"

class BooleanSensor: public Sensor{

    private:
    
        // The Arduino digital pin measuring the signal
        int _pin;
        
    public:
        
        // If true, will read the pin using analogRead() instead of digitalRead().
        // This is necessary on some chips, like the Arduino Uno*Pro, where the analog pins don't work properly with digitalRead().
        bool use_analog = false;
    
        ChangeTracker<bool> value = ChangeTracker<bool>(false);

        BooleanSensor(const int &pin){
            _pin = pin;
            pinMode(_pin, INPUT);
        }

        BooleanSensor(const int &pin, const bool pullup){
            _pin = pin;
            set_pullup(pullup);
        }
        
        void set_pullup(bool onoff){
            if (onoff) {
                pinMode(_pin, INPUT_PULLUP);
            } else {
                pinMode(_pin, INPUT);
            }
        }

        virtual void update(){
            if(use_analog){
                value.set(analogRead(_pin) >= 512);
            }else{
                value.set(digitalRead(_pin));
            }
        }
        
        bool get_value(){
            return value.get();
        }
        
        bool get_bool(){
            return get_value();
        }

        virtual bool get_and_clear_changed(){
            return value.get_and_clear_changed();
        }
        
};

#endif
