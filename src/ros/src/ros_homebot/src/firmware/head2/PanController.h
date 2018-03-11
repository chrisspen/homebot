#include <math.h>
//#include <PID_v1.h>

#include "Arduino.h"

#include "ChangeTracker.h"

// This is the speed used when rotating the head to find the centermark.
// Speed is in the range of [-255:+255].
#define PM_CALIBRATION_SPEED 200

#define PM_MAX_SPEED 255

// The number of pulses the pan motor's encoder will give to rotate the head a full 360 degrees.
// This number was calculated through the size of the neck gears, motor gearing, and encoder
// and confirmed empirically.
#define PM_FULL_REV_COUNT 1732
//866=>180
//24=>5

// If we estimate we're in this number of degrees from target, we'll cut power.
#define PM_FLEX_DEGREES 2

// Waiting to clear the centermark sensor.
#define VC_WAITING_TO_CLEAR 0

// Waiting to see the centermark sensor.
#define VC_WAITING_TO_SEE 1

// Waiting to come to a complete stop.
#define VC_WAITING_TO_STOP 2

class PanController{

    private:
    
        // 0:255
        int _speed = 0;
        
        int _calibration_speed = PM_CALIBRATION_SPEED;
        
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
        
        // The second to last time we received an encoder count update.
        // This and the last will be used to estimate current velocity.
        unsigned long _time_count_last_updated = 0;
        unsigned long _time_count_last_updated0 = 0;

        // The time when we issued a stop command.
        unsigned long _velocity_stop_time0 = 0;
        
        // The angle we want to get to.
        // -1=unset, should always be positive angle in degrees, 0:359
        float _target_angle = 0;
        
        // The current latched value of the centermark sensor.
        // The raw analog value would have to move above or below
        bool _centermark_latch = false;

        // Set to true if we've seen the centermark at least once since the system came online,
        // and we can therefore calculate absolute rotation using the encoder count.
        // Set to false otherwise.
        bool _centermark_checked = false;
        
        bool _max_velocity_checked = false;
        
        int _velocity_calibration_state = VC_WAITING_TO_CLEAR;
        
        // If true, enforces position, false, goes limp, allows manual positioning.
        bool _power = false;
        
        // If true, the target angle is currently being sought. Otherwise false.
        // Used with _power to determine if we need to cut power after establishing position.
        bool _seeking = false;
        
        unsigned long _total_ticks_counter = 0;
        
        // The total milliseconds it takes to stop moving when at full speed.
        unsigned long _total_time_to_stop = 0;
        
        // The total ticks that will be covered between issuing the stop command to fully stopping when at full speed.
        unsigned long _total_ticks_to_stop = 0;
        
        // Tracks when we start a position seeking operation, so we know when to timeout.
        unsigned long _seek_start_time = 0;
        
        int overshoot_compensation = 0;

    public:
    
        // PID variables.
        double kp=-10, kd=0.4, ki=0;
        int pos_err = 0, td_err = 0, sum_err = 0;
        float last_actual_angle = 0;
        //unsigned long _td_last_count = 0, _td_last_time = 0;
        //_td_last_count = _count, _td_last_time = millis();

        bool error_pending = false;

        // Whether or not the sensor detects the centermark reference point.
        ChangeTracker<bool> centermark = ChangeTracker<bool>(false);
        
        // The angle we currently believe we're at.
        // -1=unknown, should always be positive angle in degrees, 0:359
        ChangeTracker<float> actual_angle = ChangeTracker<float>(-1, 10, 100);//500, 1000);

        // Reports true when we've seen the centermark at least once.
        ChangeTracker<bool> calibrated = ChangeTracker<bool>(false);
        
        // If true, updates position when set. Otherwise, does nothing.
        bool active = false;

        PanController(int enable_pin, int phase_pin, int sensor_a_pin){
            _sensor_a_pin = sensor_a_pin;
            _enable_pin = enable_pin;
            _phase_pin = phase_pin;
            
            // Enable 20k pullup resistor.
            // https://www.arduino.cc/en/Reference/Constants
            // A pin may also be configured as an INPUT with pinMode(), and subsequently made HIGH with digitalWrite(). This will enable the internal 20K pullup resistors, which will pull up the input pin to a HIGH reading unless it is pulled LOW by external circuitry. 
            pinMode(_sensor_a_pin, INPUT);
            digitalWrite(_sensor_a_pin, HIGH);
            
            pinMode(_phase_pin, OUTPUT);
            pinMode(_enable_pin, OUTPUT);
            
        }
        
        unsigned long get_total_time_to_stop(){
            return _total_time_to_stop;
        }
        
        
        unsigned long get_total_ticks_to_stop(){
            return _total_ticks_to_stop;
        }

        bool is_seeking(){
            return _seeking;
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
            _seek_start_time = millis();
            error_pending = true;
            overshoot_compensation = 0;
            
            if(is_calibrated()){
                //if actual=90 and target=0 => overshoot_comp=2
                //if actual=0 and target=90 => overshoot_comp=88
                if(abs(actual_angle.get_latest() - _target_angle) > 45){

                    float change_angle = actual_angle.get_latest() - _target_angle;
                    // 90-0 = 90
                    // 0-90 = -90
                    if(change_angle > 0 && abs(change_angle - 360) < abs(change_angle)){
                        change_angle = change_angle - 360;
                    }else if(change_angle < 0 && abs(change_angle + 360) < abs(change_angle)){
                        change_angle = change_angle + 360;
                    }
                    
                    // + => CCW
                    // - => CW
                    if(change_angle > 0){
                        overshoot_compensation = 2;
                    }else{
                        overshoot_compensation = -2;
                    }

                }
                
            }

            // Reset the safety timeout clock.
            _time_count_last_updated0 = _time_count_last_updated = millis();
            
            // Reset PID session variables.
            pos_err = 0;
            td_err = 0;
            sum_err = 0;
            last_actual_angle = 0;
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
            _total_ticks_counter += ticks;
            if(actual_angle.get_latest() != -1 && is_calibrated()){

                // Update the absolute pan angle in relation to our reference position.
                _count += ticks * direction;
                tmp_angle = fmod((((float)_count)/((float)PM_FULL_REV_COUNT)*360.), 360.);
                if(tmp_angle < 0){
                    tmp_angle = 360 + tmp_angle;
                }
                actual_angle.set(tmp_angle);

                // Auto-stop once we've reached the goal.
                //if(_seeking){
                    //if(int(actual_angle.get_latest()) == int(_target_angle)){
                        //stop();
                    //}
                //}

            }
        }

        void calibrate(){
            active = true;
            
            _centermark_checked = false;
            
            _max_velocity_checked = false;
            _velocity_calibration_state = VC_WAITING_TO_CLEAR;
            _velocity_stop_time0 = 0;
            _time_count_last_updated0 = 0;
        }
        
        bool is_center_calibrated(){
            return _centermark_checked;
        }
        
        bool is_velocity_calibrated(){
            return _max_velocity_checked;
        }

        bool is_calibrated(){
            return is_center_calibrated();// && is_velocity_calibrated();
        }

        void go_to_center(){
            // Initiates the action to rotate the head until it's aligned with the centermark
            // reference position.
            set_target_angle(0);
        }

        bool is_centermark(){
            return !digitalRead(_sensor_a_pin); // 1=not centered, 0=centered
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
        
        void update_centermark() {
            // Refreshes and publishes the current centermark value.
            centermark.set(is_centermark());
            if(centermark.get_latest()){
                // When we're centered, the pan angle is 0, so incase there were rounding
                // errors during rotation, reset all counts back to zero.
                actual_angle.set_now(0);
                _count = 0;
                _centermark_checked = true;
            }
        }
        
        float get_angle_error(){
            return actual_angle.get_latest() - _target_angle;
        }

        void update(){

            update_centermark();

            // Update calibration reporting.
            calibrated.set(is_calibrated());

            if(!active){
                _calibration_speed = PM_CALIBRATION_SPEED;
            }else if(!is_center_calibrated()){
                // If we're uncalibrated, begin a dumb centermark search.
                set_speed(_calibration_speed);
            //}else if(!is_velocity_calibrated()){
                //// Measure how long it takes us to stop after obtaining full speed.
                //// We need this in order to time our deceleration so we don't overshoot our target angle when seeking.
                //if(_velocity_calibration_state == VC_WAITING_TO_CLEAR){
                    //if(!is_centermark()){
                        //_velocity_calibration_state = VC_WAITING_TO_SEE;
                    //}else{
                        //set_speed(_calibration_speed);
                    //}
                //}else if(_velocity_calibration_state == VC_WAITING_TO_SEE){
                    //if(is_centermark()){
                        //_velocity_calibration_state = VC_WAITING_TO_STOP;
                    //}else{
                        //stop();
                        //_total_ticks_counter = 0;
                        //_velocity_stop_time0 = millis();
                    //}
                //}else if(_velocity_calibration_state == VC_WAITING_TO_STOP){
                    //if(millis() - _time_count_last_updated >= 1000){
                        //// We have stopped when the encoder registers no change for 1 second.
                        //// Finalize velocity estimates.
                        //_max_velocity_checked = true;
                        //_total_time_to_stop = _time_count_last_updated - _velocity_stop_time0;
                        //_total_ticks_to_stop = _total_ticks_counter;
                    //}
                //}
            }else if(_seeking){
                // Otherwise update the current target angle using our calibrated positioning.
                _calibration_speed = PM_CALIBRATION_SPEED;

                float aa = actual_angle.get_latest(); // where we currently are
                float ta = _target_angle + overshoot_compensation; // where we want to be

                // Calculate shortest angle.
                // CW, -dir
                // CCW, +dir
                float change_angle = aa - ta;
                if(change_angle > 0 && abs(change_angle - 360) < abs(change_angle)){
                    change_angle = change_angle - 360;
                }else if(change_angle < 0 && abs(change_angle + 360) < abs(change_angle)){
                    change_angle = change_angle + 360;
                }

                //int angle_dir;
                //if(change_angle > 0){
                //    angle_dir = +1;
                //}else{
                //    angle_dir = -1;
                //}

                // Update PID state variables.
                pos_err = -change_angle; // negative is to compensate for opposite polarity in speed vs encoder direction
                td_err = aa - last_actual_angle;
                sum_err += pos_err;

                // Calculate final PID output.
                set_speed(constrain(kp*pos_err + kd*td_err + ki*sum_err, -254.0, +254.0));

                // After 2 seconds of trying to move with no change in position, abort motor
                // movement because we've probably stalled, either because we're real close
                // to the goal or there's something block us.
                if(millis() - _time_count_last_updated > 1000){
                    stop();
                }
                
                // If we're still moving after a long time, then assume something bad has happened and stop.
                // This usually means the PID controller is oscillating without convergence.
                // All pan movements should be very quick and complete in under a second.
                if(millis() - _seek_start_time > 3000){
                    stop();
                }

                // Primitive P controller.
                /*
                float final_angle = abs(change_angle);
                //int min_speed = 110; // too slow
                int min_speed = 175; // too slow
                int max_speed = 250;
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
                */
                
                // Record last tick count and time, so we can compare it to the current tick count and time to estimate velocity.
                //_td_last_count = _count;
                //_td_last_time = millis();
                last_actual_angle = aa;
                
            }else{
                _calibration_speed = PM_CALIBRATION_SPEED;
                stop();
            }

        }

};
