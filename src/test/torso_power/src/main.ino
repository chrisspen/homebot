
#define EDGE_L_PIN                  2
#define EDGE_M_PIN                  3
#define EDGE_R_PIN                  4
#define BUMPER_L_PIN                5
#define BUMPER_M_PIN                6
#define BUMPER_R_PIN                7
#define SONIC_L_PIN                 8
#define SONIC_M_PIN                 9
#define SONIC_R_PIN                 10
#define POWER_OFF_PIN               11
#define SIGNAL_BUTTON_PIN           12
#define STATUS_LED_PIN              13
#define BATTERY_VOLTAGE_PIN         A0
#define BATTERY_TEMP_PIN            A1
#define EXTERNAL_POWER_SENSE_PIN    A2

class BatteryVoltageSensor{

    private:
    
        // The expected voltage when battery is fully charged.
        float _full_voltage;
        
        // The ratio of the full voltage below which the battery is considered dead.
        float _dead_ratio;
        
        // The value of the upper resistor in the voltage divider. 
        unsigned long _r1;
        
        // The value of the lower resistor in the voltage divider.
        unsigned long _r2;
        
        // The Arduino analog pin measuring the voltage.
        int _pin;
        
        // moving average sum of voltage readings
        float _measurements;
        
        // When calculating the moving average, the weight to use on the old value.
        float _update_ratio;
        
        unsigned long _last_update;
        
        unsigned long _min_update_period;
        
        unsigned long _measurement_count;
    
    public:

        BatteryVoltageSensor(float full_voltage, long r1, long r2, int pin, float dead_ratio){
            _full_voltage = full_voltage;
            _r1 = r1;
            _r2 = r2;
            _pin = pin;
            
            _dead_ratio = dead_ratio;
            
            _update_ratio = 0.75;
            
            _last_update = 0;
            
            _min_update_period = 1000; //ms
            
            _measurement_count = 0;
            
            pinMode(_pin, INPUT);
        }
        
        void update(){
            // Read the analog pin and update the moving average.
            if((millis() - _last_update) > _min_update_period){
                int raw = analogRead(_pin);
                _measurements = (_update_ratio * _measurements) + ((1 - _update_ratio) * raw);
                _last_update = millis();
                _measurement_count += 1;
            }
        }
        
        float get_voltage(){
            // Convert the raw moving average to a voltage.
            
            if(_measurement_count < 30){
                return 0;
            }
            
            // First start by calculating the pin voltage from the ADC measurements.
            // Raw value is betweenn 0=0V and 1023=5V.
            // system_voltage/adc_resolution = analog_voltage/adc_reading
            float pin_voltage = 5./1023. * _measurements;
            
            // Then estimate the battery voltage from the pin voltage using the voltage divider.
            // Vout = Vin * R2/(R1 + R2) => Vin = Vout * (R1 + R2)/R2
            float battery_voltage = pin_voltage * (_r1 + _r2)/_r2;
            
            return battery_voltage;
        }
        
        float get_charge_ratio(){
            // Calculates the ratio representing the amount of charge before the battery level
            // reaches the dead ratio.
            float bv_ratio = get_voltage()/_full_voltage;
            return (bv_ratio - _dead_ratio)/(1 - _dead_ratio);
        }

};

unsigned long power_button_pressed_timestamp;
int power_state; // 0=power on, 1=begin shutdown, 2=shutdown

BatteryVoltageSensor battery_voltage_sensor = BatteryVoltageSensor(
        // The battery consists of 3 * 4.2V lipo cells.
        12.6,
        
        2000000, // ohms
        1000000, // ohms
        
        BATTERY_VOLTAGE_PIN,
        
        // The battery is considered dead when 80% discharged.
        0.8
    );

void setup()
{
    
    Serial.begin(57600); // must match ino.ini

    pinMode(SIGNAL_BUTTON_PIN, INPUT_PULLUP);
    
    digitalWrite(STATUS_LED_PIN, HIGH);
    pinMode(STATUS_LED_PIN, OUTPUT);
    
    digitalWrite(POWER_OFF_PIN, LOW);
    pinMode(POWER_OFF_PIN, OUTPUT);
    
    //pinMode(11, OUTPUT);
    
    power_state = 0;

}

void loop()
{
    bool sb = !digitalRead(SIGNAL_BUTTON_PIN);
    battery_voltage_sensor.update();
    float battery_voltage = battery_voltage_sensor.get_voltage();
    float battery_charge = battery_voltage_sensor.get_charge_ratio();
    
    Serial.println(String("SIGNAL_BUTTON: ") + sb);
    //https://www.arduino.cc/en/Tutorial/StringConstructors
    Serial.println(String("BATTERY_VOLTAGE: ") + String(battery_voltage, 2));
    Serial.println(String("BATTERY_VOLTAGE: ") + String(battery_charge*100));
    
    Serial.println(((String)"state: ") + power_state);
    if(power_state == 0){
        digitalWrite(STATUS_LED_PIN, HIGH);
    
        // When button's pressed, begin shutdown procedure.
        if(sb){
            power_state = 1;
            power_button_pressed_timestamp = millis();;
        }
    
    }else if(power_state == 1){
    
        if(sb){
            // While button is held down for less than 5 seconds...
            if((millis() - power_button_pressed_timestamp) < 5000){
                // Toggle LED.
                digitalWrite(STATUS_LED_PIN, (bool)(((millis() - power_button_pressed_timestamp)/500) % 2));
            }else{
                // Then do actual shutdown.
                power_state = 2;
            }
        }else{
            // Otherwise, if the button is released before shutdown, then abort.
            power_state = 0;
        }
    
    }else if(power_state == 2){

        // Signal ready to shutdown.
        digitalWrite(STATUS_LED_PIN, LOW);
            
        // Wait for release.
        if(!sb){
            power_state = 3;
        }
    
    }else if(power_state == 3){

        // Shutdown.
        digitalWrite(POWER_OFF_PIN, HIGH);
        
    }
    
    delay(500);
    
    /*
    digitalWrite(11, HIGH);
    delay(1000);
    digitalWrite(11, LOW);
    delay(1000);
    */

}
