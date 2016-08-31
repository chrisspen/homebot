
#include "Arduino.h"

#include "ID.h"
#include "ChangeTracker.h"
#include "Sensor.h"

class BumperSensor: public Sensor{
	// The bumper/contact sensor is a simple SPST switch, normally open.
	// A pullup resistor in the Arduino is used to default the value to high
	// when the switch is open.
	// When the switch is closed, the value goes to low.

    private:
    
        // The Arduino analog pin measuring the signal
        int _pin;
        
        int _index;
        
    public:
        
        ChangeTracker<bool> value = ChangeTracker<bool>(false);

        BumperSensor(int pin, int index){
            _pin = pin;
            _index = index;
            pinMode(_pin, INPUT_PULLUP);
        }
        
        virtual void update(){
            // Read the analog pin and update the moving average.
            value.set(!digitalRead(_pin));
        }
        
        bool has_contact(){
            return value.get();
        }

        virtual bool get_and_clear_changed(){
        	return value.get_and_clear_changed();
        }
        
        virtual String get_reading_packet(){
            return String(ID_GET_VALUE)+String(' ')+
                String(ID_BUMPER)+String(_index)+String(' ')+
                String(has_contact());
        }
        
};
