
#include "Arduino.h"

#include "StatusButtonSensor.h"
#include "StatusLightController.h"

#define POWERCONTROLLER_STANDBY 0
#define POWERCONTROLLER_WAITING 1
#define POWERCONTROLLER_READY 2
#define POWERCONTROLLER_SHUTDOWN 3

class PowerController{

    private:
    
        // The Arduino analog pin measuring the signal
        int _pin;
        
        bool _value;
        
        int _power_state = POWERCONTROLLER_STANDBY;
        
        unsigned long _power_button_pressed_timestamp = 0;
        
        StatusButtonSensor _status_button;
        
        StatusLightController _status_light;
        
    public:

        PowerController(int pin, StatusButtonSensor &status_button, StatusLightController &status_light){
            _pin = pin;
            _status_button = status_button;
            _status_light = status_light;
            digitalWrite(_pin, LOW);
            pinMode(_pin, OUTPUT);
        }
        
        void shutdown(){
            // Power is on as long as the pin is low.
            // Power is cut when the pin goes high.
            digitalWrite(_pin, HIGH);
        }
        
        void update(){
            _status_button.update();
            bool is_pressed = _status_button.is_pressed();
            
            if(_power_state == POWERCONTROLLER_STANDBY){
                _status_light.turn_on();
            
                // When button's pressed, begin shutdown procedure.
                if(is_pressed){
                    _power_state = POWERCONTROLLER_WAITING;
                    _power_button_pressed_timestamp = millis();
                }
            
            }else if(_power_state == POWERCONTROLLER_WAITING){
            
                if(is_pressed){
                    // While button is held down for less than 5 seconds...
                    if((millis() - _power_button_pressed_timestamp) < 5000){
                        // Toggle LED.
                        _status_light.set_state((bool)(((millis() - _power_button_pressed_timestamp)/500) % 2));
                    }else{
                        // Then do actual shutdown.
                        _power_state = POWERCONTROLLER_READY;
                    }
                }else{
                    // Otherwise, if the button is released before shutdown, then abort.
                    _power_state = POWERCONTROLLER_STANDBY;
                }
            
            }else if(_power_state == POWERCONTROLLER_READY){
        
                // Signal ready to shutdown.
                _status_light.turn_off();
                    
                // Wait for release.
                if(!is_pressed){
                    _power_state = POWERCONTROLLER_SHUTDOWN;
                }
            
            }else if(_power_state == POWERCONTROLLER_SHUTDOWN){
        
                // Shutdown.
                shutdown();
                
            }
        }
        
};
