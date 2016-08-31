
#include "Arduino.h"

#include "ID.h"
#include "ChangeTracker.h"
#include "Sensor.h"

class EdgeSensor: public Sensor{

    private:
    
        // The Arduino analog pin measuring the signal
        int _pin;
        
        int _index;
        
    public:
    
        ChangeTracker<bool> value = ChangeTracker<bool>(false);

        EdgeSensor(int pin, int index){
            _pin = pin;
            _index = index;
            pinMode(_pin, INPUT);
        }
        
        virtual void update(){
            // Read the analog pin and update the moving average.
            value.set(digitalRead(_pin));
        }
        
        bool has_contact(){
            return value.get();
        }

        virtual bool get_and_clear_changed(){
        	return value.get_and_clear_changed();
        }
        
        virtual String get_reading_packet(){
            return String(ID_GET_VALUE)+String(' ')+
                String(ID_EDGE)+String(_index)+String(' ')+
                String(has_contact());
        }
        
};
