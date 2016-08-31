
#define IR_SENSOR_A_PIN A0 // dotted
#define IR_SENSOR_B_PIN A1 // slotted
#define LED_PIN 13

#define IR_DYN_AVG_RATIO 0.9

bool led_state = false;
int sensorValue_a = 0;  // 
int sensorValue_b = 0;  // 

int threshold_a = 60;
int threshold_b = 600;

unsigned long starttime;
unsigned long last_output;

class NeckPositionSensor{

    private:
    
        // The sensor with the dot mask, sensing the center reference mark.
        int _sensor_a_pin;
        int _min_a = 1024;
        int _max_a = 0;
        int _value_a;
        bool _latch_a;
        
        // The sensor with the slotted mask, sensing general rotation.
        int _sensor_b_pin;
        int _min_b = 1024;
        int _max_b = 0;
        int _value_b;
        bool _latch_b = false;
        
        // Moving averages.
        float _dyn_avg_b_low = 0;
        float _dyn_avg_b_high = 0;
        
        unsigned long _start_time;
        bool _first;
    
    public:
        
        NeckPositionSensor(){
        }
        
        NeckPositionSensor(int sensor_a_pin, int sensor_b_pin){
            _sensor_a_pin = sensor_a_pin;
            _sensor_b_pin = sensor_b_pin;
            pinMode(_sensor_a_pin, INPUT);
            pinMode(_sensor_b_pin, INPUT);
            digitalWrite(_sensor_a_pin, HIGH);
            digitalWrite(_sensor_b_pin, HIGH);
            _start_time = millis();
            _first = true;
        }
        
        void update(){
            
            int span = 3;
            
            _value_a = analogRead(_sensor_a_pin);
            _value_b = analogRead(_sensor_b_pin);
            
            _min_a = min(_min_a, _value_a);
            _max_a = max(_max_a, _value_a);
            
            _min_b = min(_min_b, _value_b);
            _max_b = max(_max_b, _value_b);
    
            float avg_b = get_avg_b();
            int avg_b_lower = avg_b - span;
            int avg_b_upper = avg_b + span;
            
            // Update per-notch latch value.
            if(_latch_b){
                if(_value_b <= avg_b_lower){
                    _latch_b = false;
                }
            }else{
                if(_value_b >= avg_b_upper){
                    _latch_b = true;
                }
            }
    
            // Update per-notch moving averages.
            if(_latch_b){
                if(_dyn_avg_b_high){
                    _dyn_avg_b_high = _dyn_avg_b_high*(IR_DYN_AVG_RATIO) + _value_b*(1 - IR_DYN_AVG_RATIO);
                }else{
                    _dyn_avg_b_high = _value_b;
                }
            }else{
                if(_dyn_avg_b_low){
                    _dyn_avg_b_low = _dyn_avg_b_low*(IR_DYN_AVG_RATIO) + _value_b*(1 - IR_DYN_AVG_RATIO);
                }else{
                    _dyn_avg_b_low = _value_b;
                }
            }
                    
            if(_first && (_start_time + 10) < millis()){
                _first = false;
                reset_limits();
            }
            
        }
        
        void reset_limits(){
            _min_a = 1024;
            _max_a = 0;
            _min_b = 1024;
            _max_b = 0;
        }
        
        int get_a(){
            return _value_a;
        }
        
        int get_b(){
            return _value_b;
        }
        
        int get_min_a(){
            return _min_a;
        }
        
        int get_max_a(){
            return _max_a;
        }
        
        int get_avg_a(){
            return (int)((_max_a + _min_a)/2.0);
        }
        
        int get_min_b(){
            return _min_b;
        }
        
        int get_max_b(){
            return _max_b;
        }
        
        int get_avg_b(){
            int default_avg = (int)((_max_b + _min_b)/2.0);
            if(_latch_b){
                if(_dyn_avg_b_high){
                    return (int)round(_dyn_avg_b_high);
                }
            }else{
                if(_dyn_avg_b_low){
                    return (int)round(_dyn_avg_b_low);
                }
            }
            return default_avg;
        }
        
        bool is_centermark(){
            // a
            float avg_a = get_avg_a();
            return (_value_a > avg_a) ? true : false;
        }
        
        bool is_mark(){
            // b
            return _latch_b;
        }

};

NeckPositionSensor neck_position_sensor;

void setup() {

    // start serial connection
    Serial.begin(57600); // must match ino.ini
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, led_state);

    neck_position_sensor = NeckPositionSensor(IR_SENSOR_A_PIN, IR_SENSOR_B_PIN);

    starttime = millis();
    last_output = millis();

    Serial.println("");
}

void loop() {

    
    //led_state = (sensorValue < threshold)? true : false;
    //digitalWrite(LED_PIN, led_state);
    
    neck_position_sensor.update();
    
    digitalWrite(LED_PIN, neck_position_sensor.is_mark());
    
    if((last_output + 10) < millis()){
        last_output = millis();
        Serial.print(String("\r"));
        
        Serial.print(String(" a.min:")+String(neck_position_sensor.get_min_a()));
        Serial.print(String(" a.avg:")+String(neck_position_sensor.get_avg_a()));
        Serial.print(String(" a.max:")+String(neck_position_sensor.get_max_a()));
        Serial.print(String(" a.val:")+String(neck_position_sensor.get_a()));
        
        Serial.print(String(" b.min:")+String(neck_position_sensor.get_min_b()));
        Serial.print(String(" b.avg:")+String(neck_position_sensor.get_avg_b()));
        Serial.print(String(" b.max:")+String(neck_position_sensor.get_max_b()));
        Serial.print(String(" b.val:")+String(neck_position_sensor.get_b()));
        
        Serial.print(String(" centermark:")+String(neck_position_sensor.is_centermark()));
        Serial.print(String(" mark:")+String(neck_position_sensor.is_mark()));
        Serial.print("            ");
        Serial.flush();
    }
  
}
