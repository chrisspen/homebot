#ifndef BatteryVoltageSensor_h
#define BatteryVoltageSensor_h

#include "Arduino.h"
#include "ID.h"
#include "ChangeTracker.h"
#include "Sensor.h"

#define BATTERYVOLTAGE_MIN_SAMPLES 30
#define BATTERYVOLTAGE_MIN_PERIOD 1000 // ms

// Initial update takes BATTERYVOLTAGE_MIN_SAMPLES*BATTERYVOLTAGE_MIN_PERIOD/1000 seconds.
class BatteryVoltageSensor: public Sensor{

    private:
    
        // The expected voltage when battery is fully charged.
        float _full_voltage;
        
        // A value added to the measured voltage to handle measurement error.
        float _voltage_offset;
        
        // The ratio of the full voltage below which the battery is considered dead.
        float _dead_ratio;
        
        // The value of the upper resistor in the voltage divider. 
        unsigned long _r1;
        
        // The value of the lower resistor in the voltage divider.
        unsigned long _r2;
        
        // The Arduino analog pin measuring the voltage.
        int _pin;
        
        // When calculating the moving average, the weight to use on the old value.
        float _update_ratio;
        
        unsigned long _last_update;
        
        unsigned long _measurement_count;
    
    public:
    
        // moving average sum of voltage readings
        ChangeTracker<float> measurements = ChangeTracker<float>(0);

        BatteryVoltageSensor(){
        }
        
        BatteryVoltageSensor(float full_voltage, long r1, long r2, int pin, float dead_ratio, float voltage_offset=0){
            _full_voltage = full_voltage;
            _r1 = r1;
            _r2 = r2;
            _pin = pin;
            
            _dead_ratio = dead_ratio;
            
            _voltage_offset = voltage_offset;
            
            _update_ratio = 0.75;
            
            _last_update = 0;
            
            _measurement_count = 0;
            
            pinMode(_pin, INPUT);
        }
        
        virtual void update(){
            // Read the analog pin and update the moving average.
            if((millis() - _last_update) > BATTERYVOLTAGE_MIN_PERIOD){
                int raw = analogRead(_pin);
                measurements.set((_update_ratio * measurements.get()) + ((1 - _update_ratio) * raw));
                _last_update = millis();
                _measurement_count += 1;
            }
        }
        
        float get_voltage(){
            // Convert the raw moving average to a voltage.
            
            if(_measurement_count < BATTERYVOLTAGE_MIN_SAMPLES){
                return 0;
            }
            
            // First start by calculating the pin voltage from the ADC measurements.
            // Raw value is betweenn 0=0V and 1023=5V.
            // system_voltage/adc_resolution = analog_voltage/adc_reading
            float pin_voltage = 5./1023. * measurements.get();
            
            // Then estimate the battery voltage from the pin voltage using the voltage divider.
            // Vout = Vin * R2/(R1 + R2) => Vin = Vout * (R1 + R2)/R2
            float battery_voltage = pin_voltage * (_r1 + _r2)/_r2;
            
            battery_voltage += _voltage_offset;
            
            return battery_voltage;
        }
        
        float get_charge_ratio(){
            // Calculates the ratio representing the amount of charge before the battery level
            // reaches the dead ratio.
            float bv_ratio = get_voltage()/_full_voltage;
            return (bv_ratio - _dead_ratio)/(1 - _dead_ratio);
        }
        
        int get_charge_ratio_int(){
            return (int)round(get_charge_ratio()*100);
        }

        virtual bool get_and_clear_changed(){
        	//ser.write("bv.get_and_clear_changed");
        	return measurements.get_and_clear_changed();
        }
};

#endif
