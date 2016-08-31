#ifndef StatusLightController_h
#define StatusLightController_h

#include "Arduino.h"

class StatusLightController{

    private:
    
        // The Arduino analog pin measuring the signal
        int _pin;
        
        bool _state;
        
    public:

        StatusLightController(){
            _state = false;
        }
        
        StatusLightController(int pin){
            _pin = pin;
            pinMode(_pin, OUTPUT);
            _state = false;
        }
        
        void update(){
            //TODO:blink?
            digitalWrite(_pin, _state);
        }
        
        void turn_on(){
            _state = true;
            update();
        }
        
        void turn_off(){
            _state = false;
            update();
        }
        
        void toggle_state(){
            _state = !_state;
            update();
        }
        
        void set_state(bool value){
            _state = value;
            update();
        }
        
};

#endif
