
#include "Arduino.h"

class BumperSensor{

    private:
    
        // The Arduino analog pin measuring the signal
        int _pin;
        
        bool _value;
        
    public:

        BumperSensor(int pin){
            _pin = pin;
            pinMode(_pin, INPUT_PULLUP);
        }
        
        void update(){
            // Read the analog pin and update the moving average.
            _value = !digitalRead(_pin);
        }
        
        bool has_contact(){
            return _value;
        }
        
};
