#ifndef AnalogSensor_h
#define AnalogSensor_h

#include "Arduino.h"

#include "ID.h"
#include "ChangeTracker.h"
#include "Sensor.h"

class AnalogSensor: public Sensor{

    private:
    
        // The Arduino analog pin measuring the signal
        int _pin;
        
    public:
    
        ChangeTracker<int> value = ChangeTracker<int>(0);

        AnalogSensor(const int &pin){
            _pin = pin;
            pinMode(_pin, INPUT);
        }

        AnalogSensor(const int &pin, const bool pullup){
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
            // Read the analog pin and update the moving average.
            value.set(analogRead(_pin));
        }
        
        int get_value(){
            return value.get();
        }

        int get_bool(){
            if (value.get() >= 512) {
                return true;
            }else{
                return false;
            }
        }

        virtual bool get_and_clear_changed(){
        	return value.get_and_clear_changed();
        }
        
};

#endif
