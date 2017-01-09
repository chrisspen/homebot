
#include "Arduino.h"
#include "ID.h"

#include "StatusButtonSensor.h"
#include "StatusLightController.h"
#include "BatteryVoltageSensor.h"
#include "ExternalPowerSensor.h"
#include "ChangeTracker.h"
#include "SerialPort.h"
#include "Sensor.h"

#define POWERCONTROLLER_STANDBY 0
#define POWERCONTROLLER_WAITING 1
#define POWERCONTROLLER_READY 2
#define POWERCONTROLLER_SHUTDOWN 3

// 4.2*3 = 12.6
#define POWERCONTROLLER_RP_LOW 0.05 // 12.6*.8 + 12.6*.2*.05 = 10.206V
#define POWERCONTROLLER_RP_HIGH 0.95 // 12.6*.8 + 12.6*.2*.95 = 12.474V

#define POWERCONTROLLER_RP_DELAY_MS 5000

class PowerController: public Sensor{

    private:
    
        // The Arduino analog pin measuring the signal
        int _pin;
        
        bool _value;
        
        int _power_state = POWERCONTROLLER_STANDBY;
        
        unsigned long _power_button_pressed_timestamp = 0;
        
        SerialPort _ser;
        
        // The timestamp of when the recharge powerdown sequence began.
        // The actual powerdown will begin POWERCONTROLLER_RP_DELAY_MS after this.
        unsigned long _recharge_powerdown_start = 0;
        
        // Used to track sleep time.
        //unsigned long _sleeptime_ms = 0;
        //unsigned long _sleeptime_start = 0;
        
    public:

        // NOTE: Re-located here because avr-gcc crashed when this object was passed in via reference, known bug.
        StatusButtonSensor status_button = StatusButtonSensor(SIGNAL_BUTTON_PIN);
        
        StatusLightController status_light = StatusLightController(STATUS_LED_PIN);

        BatteryVoltageSensor battery_voltage_sensor = BatteryVoltageSensor(
            // The battery consists of 3 * 4.2V lipo cells.
            12.6,
            2000000, // ohms
            1000000, // ohms
            BATTERY_VOLTAGE_PIN,
            // dead_ratio, The battery is considered dead when 80% discharged.
            0.8,
            // voltage_offset, 10.9 measured by the Arduino's ADC is really 11.9 as measured by a good multimeter.
            2
        );

        ExternalPowerSensor external_power_sensor = ExternalPowerSensor(EXTERNAL_POWER_SENSE_1_PIN, EXTERNAL_POWER_SENSE_2_PIN);

        int status_light_auto = true;
        
        PowerController(int pin, SerialPort &ser){
            _pin = pin;
            digitalWrite(_pin, LOW);
            pinMode(_pin, OUTPUT);
            _ser = ser;
        }
        
        void shutdown(){
            // Power is on as long as the pin is low.
            // Power is cut when the pin goes high.
            digitalWrite(_pin, HIGH);
        }
        /*
        void sleep(unsigned long sleeptime_ms){
            // Causese head power to temporarily shutoff for the given time period.
            _sleeptime_ms = sleeptime_ms;
            _sleeptime_start = millis();
        }
        */
        virtual void update(){
            status_button.update();
            bool is_pressed = status_button.is_pressed();

            status_button.update();
            status_light.update();
            battery_voltage_sensor.update();
            external_power_sensor.update();

            if(_power_state == POWERCONTROLLER_STANDBY){
            
                if(status_light_auto){
                    status_light.on();
                }
            
                // When button's pressed, begin shutdown procedure.
                if(is_pressed){
                    _power_state = POWERCONTROLLER_WAITING;
                    _power_button_pressed_timestamp = millis();
                }
            
            }else if(_power_state == POWERCONTROLLER_WAITING){
            
                if(is_pressed){
                    // When button is held down for less than 5 seconds...
                    if((millis() - _power_button_pressed_timestamp) < 5000){
                        // Toggle LED.
                        status_light.set((bool)(((millis() - _power_button_pressed_timestamp)/500) % 2));
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
                status_light.off();
                    
                // Wait for release.
                if(!is_pressed){
                    _power_state = POWERCONTROLLER_SHUTDOWN;
                }
            
            }else if(_power_state == POWERCONTROLLER_SHUTDOWN){
        
                // Shutdown.
                shutdown();
                
            }
            
        }

        virtual bool get_and_clear_changed(){
        	return false;//status_button.value.get_and_clear_changed();
        }

        virtual String get_reading_packet(){
            return String();//status_button.get_reading_packet();
        }
        
};
