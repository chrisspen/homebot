#ifndef ArduinoTemperatureSensor_h
#define ArduinoTemperatureSensor_h

#include "ID.h"
#include "ChangeTracker.h"
#include "Sensor.h"
//#include "StringUtils.h"

// http://playground.arduino.cc/Main/InternalTemperatureSensor
class ArduinoTemperatureSensor: public Sensor{

    public:
    
        ChangeTracker<double> temperature = ChangeTracker<double>(0, 0, 1000);
    
        ArduinoTemperatureSensor(){
        }
        
        virtual void update(){
            temperature.set(get_temp());
        }
    
        double get_temp(void){
            unsigned int wADC;
            double t;
            
            // The internal temperature has to be used
            // with the internal reference of 1.1V.
            // Channel 8 can not be selected with
            // the analogRead function yet.
            
            // Set the internal reference and mux.
            ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
            ADCSRA |= _BV(ADEN);  // enable the ADC
            
            delay(20);            // wait for voltages to become stable.
            
            ADCSRA |= _BV(ADSC);  // Start the ADC
            
            // Detect end-of-conversion
            while (bit_is_set(ADCSRA, ADSC));
            
            // Reading register "ADCW" takes care of how to read ADCL and ADCH.
            wADC = ADCW;
            
            // The offset of 324.31 could be wrong. It is just an indication.
            // For the ATmega328/168 chip (e.g. Uno).
            // Does not work for the ATmega1284 (Uno*Pro).
            t = (wADC - 324.31) / 1.22;

            // The returned temperature is in degrees Celcius.
            return (t);
        }
        
        virtual bool get_and_clear_changed(){
        	return temperature.get_and_clear_changed();
        }

};

#endif
