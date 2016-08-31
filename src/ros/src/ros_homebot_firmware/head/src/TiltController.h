
#include <Servo.h>

//#include "EEPROMAnything.h"
//#include "EEPROMAddresses.h"
#include "ChangeTracker.h"
#include "Smooth.h"

// We've been told to go to a position but we're not there yet.
#define TC_STATE_SEEKING 3

// We're not enforcing or trying to achieve a position.
#define TC_STATE_LIMP 4

// The sensor reading when the head is fully tilted down.
#define TC_MIN_FEEDBACK 30

// The sensor reading when the head is fully tilted up.
#define TC_MAX_FEEDBACK 600

// The total time in milliseconds it takes to tilt from the lower to upper maximum.
// This is used to plan the timing for partial tilt changes.
#define TC_FULL_SWEEP_MILLIS 1000

// The servo degree for the head when centered vertically.
#define TC_CENTER_POSITION 90

// The number of degrees in sweep from either direction from the center.
#define TC_RANGE 90

// The maximum update frequency.
//#define TC_UPDATE_FREQ_MS 100 // max 10 times a second
#define TC_UPDATE_FREQ_MS 10 // max 100 times a second

class TiltController{

    private:
    
        Servo _servo;
    
        // The pin attached to the servo's signal wire.
        int _set_pin;
        
        // The pin attached to the servo's potentiometer.
        int _get_pin;
        
        // The current position as measured by analogRead().
        int _last_start_degrees = 0;

        // The current target position.
        int _target_degrees = TC_CENTER_POSITION;
        
        // The previous target position.
        int _last_target_degrees = 0;
        
        // Set to true if servo is enabled and being powered. Otherwise false.
        bool _attached = false;
        
        // The timestamp of when target degrees was last set.
        unsigned long _target_degrees_last_set_ms = 0;
        
        // If true, indicates we're hypotheticall moving to the target position.
        bool _moving_to_target = false;
        
        // The estimated time it should take to move from last target to current target.
        unsigned long _moving_to_target_ms = 0;
        
        // If true, enforces position, false, goes limp.
        bool _power = false;

        // The current high-level goal of the controller.
        int _state = TC_STATE_LIMP;
        
        // Timestamp of the last time the update() method was activated.
        unsigned long _last_update = 0;

    public:
        
        ChangeTracker<int> actual_degrees = ChangeTracker<int>(0, 0, 1000);

        ChangeTracker<long> raw_position = ChangeTracker<long>(0, 0, 1000);

        TiltController(int set_pin, int get_pin){

            _set_pin = set_pin; // digital pin that supports PWM
            _get_pin = get_pin; // analog
            
            pinMode(_set_pin, OUTPUT);
            pinMode(_get_pin, INPUT);
        }
        
        int get_upper_endstop_degrees(){
            return TC_CENTER_POSITION + TC_RANGE;
        }
        
        int get_lower_endstop_degrees(){
            return TC_CENTER_POSITION - TC_RANGE;
        }
        
        int get_full_sweep_degrees(){
            return get_upper_endstop_degrees() - get_lower_endstop_degrees();
        }
        
        int get_start_degrees(){
        	return _last_start_degrees;
        }

        int get_target_degrees(){
            return _target_degrees;
        }
        
        void set_target_degrees(int degrees, bool force=false){
            if(get_lower_endstop_degrees() <= degrees && degrees <= get_upper_endstop_degrees()){
                if(_last_target_degrees && !force){
                    // Calculate a delay proportional to the degree of change.
                    _moving_to_target_ms = (abs(degrees - _target_degrees)*TC_FULL_SWEEP_MILLIS)/((float)get_full_sweep_degrees());
                }else{
                    // If we're uncalibrated, or forcing movement, assume all movements take a long time.
					_moving_to_target_ms = TC_FULL_SWEEP_MILLIS;
                }
                _moving_to_target_ms = max(_moving_to_target_ms, 50);
                _last_start_degrees = get_current_degrees();//start
                _last_target_degrees = _target_degrees;
                _target_degrees = degrees;//end
                _target_degrees_last_set_ms = millis();
                _moving_to_target = true;
            }
            start();
        }
        
        bool is_moving_to_target(){
        	// Estimates of the servo is still moving based on its fastest possible speed.
        	// TODO: update to reflect smooth motion controller?
            if(_target_degrees_last_set_ms && _moving_to_target){
                if(_target_degrees_last_set_ms + _moving_to_target_ms <= millis()){
                    _moving_to_target = false;
                }else{
                    _moving_to_target = true;
                }
            }else{
                _moving_to_target = false;
            }
            return _moving_to_target;
        }
        
        bool has_arrived_at_target(){
        	return abs(get_current_degrees() - get_target_degrees()) < 3;
        }

        void wait_for_target(){
            // Blocks until we stop moving.
            while(is_moving_to_target()){
                delay(1);
            }
        }
        
        long get_current_position(){
        	return raw_position.get_latest();
        }

        long _get_current_position(){
        	return analogRead(_get_pin);
        }

        int get_current_degrees(){
            return map(
            	get_current_position(),
				TC_MIN_FEEDBACK,
                TC_MAX_FEEDBACK,
                get_lower_endstop_degrees(),
                get_upper_endstop_degrees()
            );
        }

        int get_target_position(){
            return map(
                _target_degrees,
                get_lower_endstop_degrees(),
                get_upper_endstop_degrees(),
                TC_MIN_FEEDBACK,
                TC_MAX_FEEDBACK
            );
        }
        
        int get_lower_endstop_position(){
            return map(
                get_lower_endstop_degrees(),
                get_lower_endstop_degrees(),
                get_upper_endstop_degrees(),
                TC_MIN_FEEDBACK,
                TC_MAX_FEEDBACK
            );
        }
        
        int get_upper_endstop_position(){
            return map(
                get_upper_endstop_degrees(),
                get_lower_endstop_degrees(),
                get_upper_endstop_degrees(),
                TC_MIN_FEEDBACK,
                TC_MAX_FEEDBACK
            );
        }
        
        void start(){
			_state = TC_STATE_SEEKING;
            enable();
        }
        
        void enable(){
            _servo.attach(_set_pin);
            if(!_attached){
                _servo.write(_target_degrees);
            }
            _attached = true;
        }
        
        void disable(){
            _state = TC_STATE_LIMP;
            _servo.detach();
            _attached = false;
        }
        
        void stop(){
            disable();
        }
        
        void set_power(bool value){
            _power = value;
        }
        
        bool get_power(){
            return _power;
        }
        
        Servo get_servo(){
            return _servo;
        }
        
        int get_state(){
            return _state;
        }
        
        void go_to_center(){
            set_target_degrees(TC_CENTER_POSITION, true);
        }
        
        void go_to_lower_endstop(){
            set_target_degrees(get_lower_endstop_degrees(), true);
        }
        
        void go_to_upper_endstop(){
            set_target_degrees(get_upper_endstop_degrees(), true);
        }
        
        void update(){
        
        	if(millis() - _last_update < TC_UPDATE_FREQ_MS)
        		return;

        	_last_update = millis();

        	raw_position.set(_get_current_position());

            if(TC_STATE_LIMP == _state){
            
                disable();
            
            }else if(TC_STATE_SEEKING == _state){

                // Tell the servo to move directly to the target position.
                // Simple but very fast and sudden.
//                if(!has_arrived_at_target() && is_moving_to_target()){
//            	    _servo.write(_target_degrees);
//            	}else{
//            	    actual_degrees.set(_target_degrees);
//            	    if(!_power){
//            	        disable();
//            	    }
//            	}


            	// Move the servo smoothly.
            	//TODO:fix? too jerky
            	unsigned long t = millis() - _target_degrees_last_set_ms;
        	    int v = get_servo_signal(
					get_start_degrees(),//start
					get_target_degrees(),//end
        			50,//speed
        			t/1000.//t
        		);
				v = max(v, get_lower_endstop_degrees());
				v = min(v, get_upper_endstop_degrees());
    	    	_servo.write(v);
        	    if(v == get_target_degrees()){
					actual_degrees.set(_target_degrees);
					if(!_power){
						disable();
					}
        	    }


            }else{

                disable();

            }
            
        }

};
