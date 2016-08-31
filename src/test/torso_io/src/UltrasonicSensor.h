
#include "Arduino.h"

class UltrasonicSensor
{
    private:
        
        // pin number of Arduino that is connected with SIG pin of Ultrasonic Ranger.
        int _pin;
        
        // the Pulse time received;
        long _duration;
        
    public:
        UltrasonicSensor(int pin);
        void update(void);
        long microsecondsToCentimeters(void);
        long microsecondsToInches(void);
};

UltrasonicSensor::UltrasonicSensor(int pin)
{
    _pin = pin;
}

/* Begin the detection and get the pulse back signal */
void UltrasonicSensor::update(void)
{
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
    
    delayMicroseconds(2);
    
    digitalWrite(_pin, HIGH);
    
    delayMicroseconds(5);
    
    digitalWrite(_pin, LOW);
    pinMode(_pin, INPUT);
    _duration = pulseIn(_pin, HIGH);
}

/* The measured distance from the range 0 to 400 Centimeters */
long UltrasonicSensor::microsecondsToCentimeters(void)
{
    return _duration/29/2;   
}

/* The measured distance from the range 0 to 157 Inches */
long UltrasonicSensor::microsecondsToInches(void)
{
    return _duration/74/2;
}
