#ifndef ExternalPowerSensor_h
#define ExternalPowerSensor_h

#include "Arduino.h"

#include "ID.h"
#include "ChangeTracker.h"
#include "Sensor.h"

class ExternalPowerSensor: public Sensor{

    private:

		// The pin measuring immediately after the external connector but before the reed switch.
		// Reads high when the external power plug is connected and external power is present.
        int _pin1; // EP1

        // The pin measuring behind the reed switch.
		// Reads high when the external power plug is connected, regardless of whether or not
        // external power is present.
        // This lets the robot know if it's connected to a dead recharge station.
        int _pin2; // EP2
        
    public:

        ChangeTracker<bool> value1 = ChangeTracker<bool>(false);
        ChangeTracker<bool> value2 = ChangeTracker<bool>(false);
        
        ExternalPowerSensor(){
        }
        
        ExternalPowerSensor(int pin1, int pin2){
            _pin1 = pin1;
            pinMode(_pin1, INPUT);
            _pin2 = pin2;
            pinMode(_pin2, INPUT);
        }
        
        virtual void update(){
            // Read the analog pin and update the moving average.
            value1.set(digitalRead(_pin1));
            value2.set(digitalRead(_pin2));
        }

        bool external_power_present(){
            return value1.get();
        }

        bool is_physically_connected(){
            return value2.get();
        }
        
        bool external_power_connected(){
            return is_physically_connected() && external_power_present();
        }

        virtual bool get_and_clear_changed(){
        	return value1.get_and_clear_changed() || value2.get_and_clear_changed();
        }

        virtual String get_reading_packet(){
            return String(ID_GET_VALUE)+String(' ')+
                String(ID_EXTERNAL_POWER)+String(' ')+
                String(value1.get())+String(' ')+
                String(value2.get());
        }
        
};

#endif
