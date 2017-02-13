
#include "Arduino.h"

#include "ID.h"
#include "ChangeTracker.h"
#include "Sensor.h"

class BooleanSensor: public Sensor{

    private:
    
        // The Arduino analog pin measuring the signal
        int _pin;
        
    public:
    
        ChangeTracker<bool> value = ChangeTracker<bool>(false);

        BooleanSensor(const int &pin){
            _pin = pin;
            pinMode(_pin, INPUT);
        }
        
        virtual void update(){
            // Read the analog pin and update the moving average.
            value.set(digitalRead(_pin));
        }
        
        bool get_value(){
            return value.get();
        }

        virtual bool get_and_clear_changed(){
        	return value.get_and_clear_changed();
        }
        
};
