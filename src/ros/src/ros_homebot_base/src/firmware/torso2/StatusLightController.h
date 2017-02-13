#ifndef StatusLightController_h
#define StatusLightController_h

#include "Arduino.h"
#include "Sensor.h"

#define STATUSLIGHT_MODE_SOLID 1
//#define STATUSLIGHT_MODE_BLINK 2

class StatusLightController: public Sensor{

    private:
    
        // The Arduino analog pin measuring the signal
        int _pin;
        
        int _mode = STATUSLIGHT_MODE_SOLID;
        
        unsigned long _last = 0;
        //unsigned long _blink_rate = 1000;
        
    public:

        StatusLightController(){
        }
        
        StatusLightController(int pin){
            _pin = pin;
            pinMode(_pin, OUTPUT);
            digitalWrite(_pin, false);
        }
        /*
        void blink(unsigned long rate){
            _mode = STATUSLIGHT_MODE_BLINK;
            _blink_rate = rate;
        }*/
        
        virtual void update(){
            /*if(_mode == STATUSLIGHT_MODE_BLINK){
                if(millis() - _last > _blink_rate){
                    _state = !_state;
                    _last = millis();
                }
            }*/

        }
        
        virtual bool get_and_clear_changed(){
        	return false; // OVERRIDE
        }

        void on(){
            _mode = STATUSLIGHT_MODE_SOLID;
            digitalWrite(_pin, true);
            update();
        }
        
        void off(){
            _mode = STATUSLIGHT_MODE_SOLID;
            digitalWrite(_pin, false);
            update();
        }
        
        void toggle(){
            digitalWrite(_pin, !digitalRead(_pin));
            update();
        }
        
        void set(bool v){
        	digitalWrite(_pin, v);
        }

};

#endif
