
#include "Arduino.h"

#include "ID.h"
#include "ChangeTracker.h"
#include "Sensor.h"

class UltrasonicSensor: public Sensor
{
    private:
        
        // pin number of Arduino that is connected with SIG pin of Ultrasonic Ranger.
        int _pin;
        
    public:
    
        // the Pulse time received;
        ChangeTracker<int> distance = ChangeTracker<int>(0);
    
        UltrasonicSensor(const int &pin){
            _pin = pin;
        }
        
        /* Begin the detection and get the pulse back signal */
        virtual void update(void)
        {
            // Switch to output initiate sensor ping.
            pinMode(_pin, OUTPUT);

            // Clear sensor by setting low.
            digitalWrite(_pin, LOW);
            delayMicroseconds(2);
            
            // Start ping.
            digitalWrite(_pin, HIGH);
            delayMicroseconds(5);
            
            // End ping.
            digitalWrite(_pin, LOW);

            // Switch to read and begin timing while waiting for the response.
            pinMode(_pin, INPUT);

            // The measured distance from the range 0 to 400 Centimeters
            distance.set(pulseIn(_pin, HIGH)/29/2);
        }

        virtual bool get_and_clear_changed(){
        	return distance.get_and_clear_changed();
        }

};
