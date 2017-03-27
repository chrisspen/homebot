
#include "Arduino.h"
#include "ID.h"

//#include "StatusLightController.h"
//#include "BatteryVoltageSensor.h"
//#include "ExternalPowerSensor.h"
#include "ChangeTracker.h"
//#include "SerialPort.h"
#include "Sensor.h"
#include "BooleanSensor.h"

#define POWERCONTROLLER_STANDBY 0
#define POWERCONTROLLER_WAITING 1
#define POWERCONTROLLER_READY 2
#define POWERCONTROLLER_SHUTDOWN 3

// 4.2*3 = 12.6
#define POWERCONTROLLER_RP_LOW 0.05 // 12.6*.8 + 12.6*.2*.05 = 10.206V
#define POWERCONTROLLER_RP_HIGH 0.95 // 12.6*.8 + 12.6*.2*.95 = 12.474V


class PowerController: public Sensor{

    private:
        
        unsigned long _power_button_pressed_timestamp = 0;
        
        bool is_pressed = false;

    public:

        ChangeTracker<int> power_state = ChangeTracker<int>(POWERCONTROLLER_STANDBY);

        PowerController(){
            // We keep the pin low to keep ourselves on.
            // It'll be pulled high to shutdown.
            digitalWrite(POWER_OFF_PIN, LOW);
            pinMode(POWER_OFF_PIN, OUTPUT);
        }
        
        void shutdown(){
            // Power is on as long as the pin is low.
            // Power is cut when the pin goes high.
            digitalWrite(POWER_OFF_PIN, HIGH);
        }

        bool is_idle(){
        	return power_state.get_latest() == POWERCONTROLLER_STANDBY;
        }

        virtual void update(){
            is_pressed = !digitalRead(SIGNAL_BUTTON_PIN);
            if(power_state.get_latest() == POWERCONTROLLER_STANDBY){
            	// Default state. Power button is not being used.
            
                // When button's pressed, begin shutdown procedure.
                if(is_pressed){
                    power_state.set(POWERCONTROLLER_WAITING);
                    _power_button_pressed_timestamp = millis();
                }
            
            }else if(power_state.get_latest() == POWERCONTROLLER_WAITING){
            	// Power button has been pressed, and we're waiting to see how long it's being held down.
            
                if(is_pressed){
                    // When button is held down for less than 5 seconds...
                    if((millis() - _power_button_pressed_timestamp) < 5000){
                        // Toggle LED.
                        digitalWrite(STATUS_LED_PIN, (bool)(((millis() - _power_button_pressed_timestamp)/500) % 2));
                    }else{
                        // Then do actual shutdown.
                        power_state.set(POWERCONTROLLER_READY);
                    }
                }else{
                    // Otherwise, if the button is released before shutdown, then abort.
                    power_state.set(POWERCONTROLLER_STANDBY);
                }
            
            }else if(power_state.get_latest() == POWERCONTROLLER_READY){
            	// Power button has been held down long enough to cause a hard power off,
            	// so signal the shutdown is ready and wait for the button to be released.
        
                // Signal ready to shutdown.
                digitalWrite(STATUS_LED_PIN, LOW);
                    
                // Wait for release.
                if(!is_pressed){
                    power_state.set(POWERCONTROLLER_SHUTDOWN);
                }
            
            }else if(power_state.get_latest() == POWERCONTROLLER_SHUTDOWN){
            	// Button released, finalizing hard power off.
        
                // Shutdown.
                shutdown();
                
            }
        }

        virtual bool get_and_clear_changed(){
        	return power_state.get_and_clear_changed();
        }
        
};
