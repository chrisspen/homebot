
#include "Arduino.h"

#include "ID.h"
#include "ChangeTracker.h"
#include "StringUtils.h"
#include "Sensor.h"

class BatteryTemperatureSensor: public Sensor{

    private:
    
        // The Arduino analog pin measuring the signal
        int _pin;
        
    public:

        ChangeTracker<int> value = ChangeTracker<int>(0, 0, 1000);
        
        BatteryTemperatureSensor(int pin){
            _pin = pin;
            pinMode(_pin, INPUT);
        }
        
        virtual void update(){
            // Read the analog pin and update the moving average.
            value.set(analogRead(_pin));
        }
        
        double get_temperature_celcius(){
            double Temp;
            Temp = log(10000.0*((1024.0/value.get()-1))); 
        //         =log(10000.0/(1024.0/value.get()-1)) // for pull-up configuration
            Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp ))* Temp );
            Temp = Temp - 273.15;            // Convert Kelvin to Celcius
            //Temp = (Temp * 9.0)/ 5.0 + 32.0; // Convert Celcius to Fahrenheit
            return Temp;
        }
        
        double get_temperature_fahrenheit(){
            return (get_temperature_celcius() * 9.0)/ 5.0 + 32.0;
        }
        
};
