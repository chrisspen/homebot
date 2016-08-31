
#include "Arduino.h"

class ExternalPowerSensor{

    private:
    
        // The Arduino analog pin measuring the signal
        int _pin;
        
        bool _value;
        
    public:

        ExternalPowerSensor(int pin){
            _pin = pin;
            pinMode(_pin, INPUT);
        }
        
        void update(){
            // Read the analog pin and update the moving average.
            _value = digitalRead(_pin);
        }
        
        bool is_connected(){
            return _value;
        }
        
};
