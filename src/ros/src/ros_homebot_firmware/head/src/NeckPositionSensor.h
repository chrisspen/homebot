
#ifndef NeckPositionSensor_h
#define NeckPositionSensor_h

#include "Arduino.h"

#define NECKPOSITIONSENSOR_DYN_AVG_RATIO 0.9

// See models/bearings/bearing_gradient_dashed.scad for strip definition.
// Circumference = 300mm, white/black mark = 1mm => 300/2 = 150 marks
// 1 mark => 360/300 = 1.2 degrees per mark
#define NECKPOSITIONSENSOR_TOTAL_BLACK_MARKS 150
#define NECKPOSITIONSENSOR_DEGREES_PER_MARK 1.2

#define NECKPOSITIONSENSOR_NO_MOVEMENT 0
#define NECKPOSITIONSENSOR_CW_MOVEMENT 1
#define NECKPOSITIONSENSOR_CCW_MOVEMENT 2

class NeckPositionSensor{

    private:
    
        // The sensor with the dot mask, sensing the center reference mark.
        int _sensor_a_pin;
        int _min_a = 1024;
        int _max_a = 0;
        int _value_a;
        bool _latch_a;
        
        // The sensor with the slotted mask, sensing general rotation.
        //int _sensor_b_pin;
        //int _min_b = 1024;
        //int _max_b = 0;
        //int _value_b;
        //bool _latch_b = false;
        
        // Moving averages.
        float _dyn_avg_a_low = 0;
        float _dyn_avg_a_high = 0;
        //float _dyn_avg_b_low = 0;
        //float _dyn_avg_b_high = 0;
        
        unsigned long _start_time;
        bool _first;
        
        // The estimated angle of the head rotated about the z-axis.
        float _angle = 0;
    
    public:
    
        int centermark_count = 0;
        int mark_count = 0;
        
        NeckPositionSensor(){
        }
        
        NeckPositionSensor(int sensor_a_pin){
            _sensor_a_pin = sensor_a_pin;
            //_sensor_b_pin = sensor_b_pin;
            pinMode(_sensor_a_pin, INPUT);
            //pinMode(_sensor_b_pin, INPUT);
            digitalWrite(_sensor_a_pin, HIGH);
            //digitalWrite(_sensor_b_pin, HIGH);
            _start_time = millis();
            _first = true;
        }
        
        void _register_change(int motor_dir){
        }
        
        void update(int motor_dir=NECKPOSITIONSENSOR_NO_MOVEMENT){
            /*
            Parameters:
            
                motor_dir := direction we're causing the pan motor to turn the neck,
                    used to help update angle
                    0 = no expected movement
                    1 = clockwise (looking down)
                    2 = counter-clockwise (looking down)
            */
            
            int span_a = 3;
            int span_b = 3;
            
            _value_a = analogRead(_sensor_a_pin);
            //_value_b = analogRead(_sensor_b_pin);
            
            _min_a = min(_min_a, _value_a);
            _max_a = max(_max_a, _value_a);
            
            //_min_b = min(_min_b, _value_b);
            //_max_b = max(_max_b, _value_b);
    
            float avg_a = get_avg_a();
            int avg_a_lower = avg_a - span_a;
            int avg_a_upper = avg_a + span_a;
            
            //float avg_b = get_avg_b();
            //int avg_b_lower = avg_b - span_b;
            //int avg_b_upper = avg_b + span_b;
            
            // Update per-notch latch value.
            if(_latch_a){
                if(_value_a <= avg_a_lower){
                    _latch_a = false;
                }
            }else{
                if(_value_a >= avg_a_upper){
                    _latch_a = true;
                    centermark_count += 1;
                }
            }
            /*
            if(_latch_b){
                if(_value_b <= avg_b_lower){
                    _latch_b = false;
                }
            }else{
                if(_value_b >= avg_b_upper){
                    _latch_b = true;
                    mark_count += 1;
                }
            }
            */
    
            // Update per-notch moving averages.
            if(_latch_a){
                if(_dyn_avg_a_high){
                    _dyn_avg_a_high = _dyn_avg_a_high*(NECKPOSITIONSENSOR_DYN_AVG_RATIO) + _value_a*(1 - NECKPOSITIONSENSOR_DYN_AVG_RATIO);
                }else{
                    _dyn_avg_a_high = _value_a;
                }
            }else{
                if(_dyn_avg_a_low){
                    _dyn_avg_a_low = _dyn_avg_a_low*(NECKPOSITIONSENSOR_DYN_AVG_RATIO) + _value_a*(1 - NECKPOSITIONSENSOR_DYN_AVG_RATIO);
                }else{
                    _dyn_avg_a_low = _value_a;
                }
            }
            /*
            if(_latch_b){
                if(_dyn_avg_b_high){
                    _dyn_avg_b_high = _dyn_avg_b_high*(NECKPOSITIONSENSOR_DYN_AVG_RATIO) + _value_b*(1 - NECKPOSITIONSENSOR_DYN_AVG_RATIO);
                }else{
                    _dyn_avg_b_high = _value_b;
                }
            }else{
                if(_dyn_avg_b_low){
                    _dyn_avg_b_low = _dyn_avg_b_low*(NECKPOSITIONSENSOR_DYN_AVG_RATIO) + _value_b*(1 - NECKPOSITIONSENSOR_DYN_AVG_RATIO);
                }else{
                    _dyn_avg_b_low = _value_b;
                }
            }
            */
                    
            if(_first && (_start_time + 10) < millis()){
                _first = false;
                reset_limits();
            }
            
        }
        
        void reset_limits(){
            _min_a = 1024;
            _max_a = 0;
            //_min_b = 1024;
            //_max_b = 0;
        }
        
        float get_angle(){
            return _angle;
        }
        
        int get_a(){
            return _value_a;
        }
        /*
        int get_b(){
            return _value_b;
        }*/
        
        int get_min_a(){
            return _min_a;
        }
        
        int get_max_a(){
            return _max_a;
        }
        
        int get_avg_a(){
            int default_avg = (int)((_max_a + _min_a)/2.0);
            if(_latch_a){
                if(_dyn_avg_a_high){
                    return (int)round(_dyn_avg_a_high);
                }
            }else{
                if(_dyn_avg_a_low){
                    return (int)round(_dyn_avg_a_low);
                }
            }
            return default_avg;
        }
        /*
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
        */
        
        bool is_centermark(){
            // a
            //float avg_a = get_avg_a();
            //return (_value_a > avg_a) ? true : false;
            return _latch_a;
        }
        /*
        bool is_mark(){
            // b
            return _latch_b;
        }
        */

};

#endif
