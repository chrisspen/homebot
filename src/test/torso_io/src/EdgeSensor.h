
#include "Arduino.h"

class EdgeSensor{

    private:
    
        // The Arduino analog pin measuring the signal
        int _pin;
        
        bool _value;
        
    public:

        EdgeSensor(int pin){
            _pin = pin;
            pinMode(_pin, INPUT);
        }
        
        void update(){
            // Read the analog pin and update the moving average.
            _value = digitalRead(_pin);
        }
        
        bool has_contact(){
            return _value;
        }
        
        
};
