/*
Simple class that wraps an arbitrary variable, and maintains
a flag indicating when it's been changed.

Used to quickly determining when push-notifications for tracked variables
should be sent to the host.
*/

#ifndef ChangeTracker_h
#define ChangeTracker_h

template <class T>
class ChangeTracker{

    private:
    
        // Current value.
        T _el;
        
        // Next possible value after debounce period.
        T _next_el;
        
        bool _changed = false;
        
        // milliseconds, 0=no debouncing
        unsigned long _debounce = 0;
        
        unsigned long _last_change_time = 0;
        
        unsigned long _last_save_time = 0;
        
        // minimum time between changes in milliseconds
        unsigned long _rate_limit = 0;
        
    public:
    
        ChangeTracker(T v, unsigned long debounce=0, unsigned long rate_limit=0){
            _next_el = v;
            _el = v;
            _debounce = debounce;
            _last_change_time = millis();
            _rate_limit = rate_limit;
            _last_save_time = millis();
        }
        
        void update(){
        
            // Evaluate debounce period.
            if(_debounce){
                if(millis() - _debounce >= _last_change_time && _el != _next_el){
                    // Enough time has passed since the last change for debouncing to pass
                    // so register change.
                    _el = _next_el;
                    _changed = true;
                }
            }else{
                // No debounce period, just check for change.
                if(_el != _next_el){
                    _el = _next_el;
                    _changed = true;
                }
            }
         
        }
    
        void set(T v){
            if(v != _next_el){
                _next_el = v;
                _last_change_time = millis();
            }
        }
        
        // Sets the value immediately, bypassing any debounce or rate limiting.
        void set_now(T v){
            if(v != _el){
                _last_change_time = millis();
                _changed = true;
            }
            _next_el = v;
            _el = v;
        }
    
        T get(){
            update();
            return _el;
        }
    
        T get_latest(){
            return _next_el;
        }
    
        bool is_changed(){
            update();
            return _changed;
        }
        
        bool get_and_clear_changed(){
              
            if(_rate_limit){
                if(_last_save_time + _rate_limit > millis()){
                    return false;
                }
                _last_save_time = millis();
            }
            
            update();
            bool c = _changed;
            _changed = false;
            return c;
        }

};

#endif
