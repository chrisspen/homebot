
#include "Arduino.h"

#include "ID.h"
#include "ChangeTracker.h"
#include "Sensor.h"

class UltrasonicSensor: public Sensor
{
    private:
        
        // pin number of Arduino that is connected with SIG pin of Ultrasonic Ranger.
        int _pin;
        
        int _index;
        
    public:
    
        // the Pulse time received;
        ChangeTracker<long> duration = ChangeTracker<long>(0);
    
        UltrasonicSensor(int pin, int index){
            _pin = pin;
            _index = index;
        }
        
        /* Begin the detection and get the pulse back signal */
        virtual void update(void)
        {
            pinMode(_pin, OUTPUT);
            digitalWrite(_pin, LOW);
            
            delayMicroseconds(2);
            
            digitalWrite(_pin, HIGH);
            
            delayMicroseconds(5);
            
            digitalWrite(_pin, LOW);
            pinMode(_pin, INPUT);
            duration.set(pulseIn(_pin, HIGH));
        }
        
        /* The measured distance from the range 0 to 400 Centimeters */
        long microsecondsToCentimeters(void)
        {
            return duration.get()/29/2;   
        }
        
        /* The measured distance from the range 0 to 157 Inches */
        long microsecondsToInches(void)
        {
            return duration.get()/74/2;
        }

        virtual bool get_and_clear_changed(){
        	return duration.get_and_clear_changed();
        }
        
        virtual String get_reading_packet(){
            return String(ID_GET_VALUE)+String(' ')+
                String(ID_ULTRASONIC)+String(_index)+String(' ')+
                String(microsecondsToCentimeters());
        }
};
