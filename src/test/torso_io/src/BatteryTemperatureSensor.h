
#include "Arduino.h"

class BatteryTemperatureSensor{

    private:
    
        // The Arduino analog pin measuring the signal
        int _pin;
        
        int _value;
        
    public:

        BatteryTemperatureSensor(int pin){
            _pin = pin;
            pinMode(_pin, INPUT);
        }
        
        void update(){
            // Read the analog pin and update the moving average.
            _value = analogRead(_pin);
        }
        
        double get_temperature_fahrenheit(){
            double Temp;
            Temp = log(10000.0*((1024.0/_value-1))); 
        //         =log(10000.0/(1024.0/_value-1)) // for pull-up configuration
            Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp ))* Temp );
            Temp = Temp - 273.15;            // Convert Kelvin to Celcius
            Temp = (Temp * 9.0)/ 5.0 + 32.0; // Convert Celcius to Fahrenheit
            return Temp;
        }
        
};
