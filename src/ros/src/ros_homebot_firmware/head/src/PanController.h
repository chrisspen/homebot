#include <math.h>
//#include <PID_v1.h>

#include "Arduino.h"

#include "ChangeTracker.h"

// This is the speed used when rotating the head to find the centermark.
#define PM_CALIBRATION_SPEED 120

// The centermark sensor is an analog IR sensor looking for a single white stripe on a black
// background.
// The sensor is wired so that when it sees the white strip, the analog value goes to 0.
// The threshold sets the mid-point below which a positive centermark measurement is detected.
// Off readings average around 531.
// On readings average around 442.
#define PM_CENTERMARK_THRESHOLD 480

// If the centermark sensor is right on the edge of the white stripe, the readings may rapidly
// fluxuation above and below the threshold. To above rapidly changing false readings, the span
// determines the amount of hystersis needed before a change over the threshold is allowed.
#define PM_CENTERMARK_SPAN 3

// The number of pulses the pan motor's encoder will give to rotate the head a full 360 degrees.
// This number was calculated through the size of the neck gears, motor gearing, and encoder
// and confirmed empirically.
#define PM_FULL_REV_COUNT 1732

// If we estimate we're in this number of degrees from target, we'll cut power.
#define PM_FLEX_DEGREES 2

class PanController{

    private:
    
		// 0:255
        int _speed = 0;
        
        // The direction we desire to go.
        // true=CW, false=CCW
        bool _direction = true;
        
        // Pin for detecting centermark.
        int _sensor_a_pin;

        // Pin for controlling motor power through PWM.
        int _enable_pin;

        // Pin for controlling motor direction.
        int _phase_pin;
        
        // Relative count of motor encoder ticks.
        // Increments when moving clockwise.
        // Decrements when moving counter clockwise.
        long _count = 0;

        // The last time we received an encoder count update.
        unsigned long _time_count_last_updated = 0;
        
        // The angle we want to get to.
        // -1=unset, should always be positive angle in degrees, 0:359
        float _target_angle = 0;
        
        // The current latched value of the centermark sensor.
        // The raw analog value would have to move above or below
        // PM_CENTERMARK_THRESHOLD +- PM_CENTERMARK_SPAN for this to change.
        bool _centermark_latch = false;

        // Set to true if we've seen the centermark at least once since the system came online,
        // and we can therefore calculate absolute rotation using the encoder count.
        // Set to false otherwise.
        bool _centermark_checked = false;
        
        // If true, enforces position, false, goes limp, allows manual positioning.
        bool _power = false;
        
        // If true, the target angle is currently being sought. Otherwise false.
        // Used with _power to determine if we need to cut power after establishing position.
        bool _seeking = false;

    public:
    
        // Whether or not the sensor detects the centermark reference point.
        ChangeTracker<bool> centermark = ChangeTracker<bool>(false);
        
        // The angle we currently believe we're at.
        // -1=unknown, should always be positive angle in degrees, 0:359
        ChangeTracker<float> actual_angle = ChangeTracker<float>(-1, 500, 1000);

        // Reports true when we've seen the centermark at least once.
        ChangeTracker<bool> calibrated = ChangeTracker<bool>(false);
        
        PanController(int enable_pin, int phase_pin, int sensor_a_pin){
            _sensor_a_pin = sensor_a_pin;
            _enable_pin = enable_pin;
            _phase_pin = phase_pin;
            
            pinMode(_sensor_a_pin, INPUT);
            digitalWrite(_sensor_a_pin, HIGH);
            pinMode(_phase_pin, OUTPUT);
            pinMode(_enable_pin, OUTPUT);
            
        }
        
        // Set/get power.

        void set_power(bool value){
            _power = value;
        }
        
        bool get_power(){
            return _power;
        }
        
        // Set/get target angle.

        float get_target_angle(){
            return _target_angle;
        }

        void set_target_angle(float v){
            _target_angle = fmod(v, 360.);

            _seeking = true;

            // Reset the safety timeout clock.
            _time_count_last_updated = millis();
        }

        // Set/get speed.

        void set_speed(int s){
        	// Given an absolute speed in the range of [-255:+255],
        	// sets the corresponding enable and phase pins to make the pan motor spin
        	// in the desired direction at the desired speed.
            _direction = (s >= 0) ? true : false; //+=CCW,-=CW
            _speed = abs(s);

            analogWrite(_enable_pin, _speed);
            digitalWrite(PAN_MOTOR_PHASE, _direction);

        }

        int get_speed(){
            return _speed * ((_direction)?+1:-1);
        }

        int get_abs_speed(){
            return _speed;
        }

        bool get_direction(){
            return _direction;
        }

        long get_count(){
            return _count;
        }

        void set_encoder_feedback(int direction, int ticks){
            //direction := +1=CW, -1=CCW
            //tick := number of encoder pulses, PM_FULL_REV_COUNT for 360 degrees
            float tmp_angle;
            _time_count_last_updated = millis();
            if(actual_angle.get_latest() != -1 && is_calibrated()){

            	// Update the absolute pan angle in relation to our reference position.
                _count += ticks * direction;
                tmp_angle = fmod((((float)_count)/((float)PM_FULL_REV_COUNT)*360.), 360.);
                if(tmp_angle < 0){
                    tmp_angle = 360 + tmp_angle;
                }
                actual_angle.set(tmp_angle);

            }
        }

        bool is_calibrated(){
            return _centermark_checked;
        }

        void go_to_center(){
            // Initiates the action to rotate the head until it's aligned with the centermark
        	// reference position.
        	set_target_angle(0);
        }

        int raw_centermark(){
        	return analogRead(_sensor_a_pin);
        }

        bool is_centermark(){
            // Reads the centermark sensor and sets the internal latched value,
        	// but does not update the publicly reported centermark value.

        	// To help guard against false positives, once we've confirmed the centermark,
        	// don't bother re-checking until we know we're close.
        	if(_centermark_checked && (actual_angle.get_latest() > 10 || actual_angle.get_latest() < 350)){
        		return false;
        	}

            int v = raw_centermark();
            if(_centermark_latch){
                if(v >= PM_CENTERMARK_THRESHOLD + PM_CENTERMARK_SPAN){
                    _centermark_latch = false;
                }
            }else{
                if(v <= PM_CENTERMARK_THRESHOLD - PM_CENTERMARK_SPAN){
                    _centermark_latch = true;
                }
            }
            return _centermark_latch;
        }

        void stop(){
            // Halt pan motor.
            set_speed(0);
            if(!_power){
            	// If power steering isn't enabled and we've achieved the target,
            	// then kill power and allow for manual repositioning.
            	_seeking = false;
            }
        }

        void update(){

        	// Refreshes and publishes the current centermark value.
            centermark.set(is_centermark());
            if(centermark.get_latest()){
            	// When we're centered, the pan angle is 0, so incase there were rounding
            	// errors during rotation, reset all counts back to zero.
                actual_angle.set_now(0);
                _count = 0;
                _centermark_checked = true;
            }

            // Update calibration reporting.
            calibrated.set(is_calibrated());

            if(!is_calibrated()){
                // If we're uncalibrated, begin a dumb centermark search.
            	set_speed(PM_CALIBRATION_SPEED);
            }else if(_seeking){
            	// Otherwise update the current target angle using our calibrated positioning.

				float aa = actual_angle.get_latest();
				float ta = _target_angle;

				// CW, -dir
				// CCW, +dir

				// Calculate shortest angle.
				float change_angle = aa - ta;
				if(change_angle > 0 && abs(change_angle - 360) < abs(change_angle)){
					change_angle = change_angle - 360;
				}else if(change_angle < 0 && abs(change_angle + 360) < abs(change_angle)){
					change_angle = change_angle + 360;
				}

				int angle_dir;
				if(change_angle > 0){
					angle_dir = +1;
				}else{
					angle_dir = -1;
				}

				// Primitive P controller.
				float final_angle = abs(change_angle);
				int min_speed = 110; // too slow
				//int max_speed = 175; // too slow
				//int min_speed = 125; // too slow
				int max_speed = 200; // too slow
				int min_angle = 0;
				int max_angle = 180;
				if(millis() - _time_count_last_updated > 2000){
					// After 2 seconds of trying to move with no change in position, abort motor
					// movement because we've probably stalled, either because we're real close
					// to the goal or there's something block us.
					stop();
				}else if(final_angle <= PM_FLEX_DEGREES){
					// If we're close enough, stop because we're not accurate enough to precisely
					// stop under a degree.
					stop();
				}else{
					// Otherwise set speed proportional to the distance to the goal.
					// Use the top speed if we're far away, and slow down the closer we get.
					set_speed(map(final_angle, min_angle, max_angle, min_speed, max_speed)*angle_dir);
				}
            }else{
				stop();
            }

        }

};
