#ifndef SerialPort_h
#define SerialPort_h

#include "ID.h"

#define BLANK_ID String("")

#define HEAD "HEAD"
#define TORSO "TORSO"

#define MOTOR_FORWARD "forward"
#define MOTOR_REVERSE "reverse"
#define MOTOR_TURN_CW "turn_cw"
#define MOTOR_TURN_CCW "turn_ccw"
#define MOTOR_BREAK "break"

#define OK "OK"
#define PONG "PONG"

class Packet{

    private:
        String _id;
        String _data;

    public:
    
        int arg_length;
        
        Packet(String id, String data){
            _id = id;
            _data = data;
            
            arg_length = (_data.length())?1:0;
            for(int i=0; i<_data.length(); i++){
                if(_data.charAt(i) == ' '){
                    arg_length += 1;
                }
            }
        }

        bool is_valid(){
        	return _id != BLANK_ID;
        }
        
        char get_id(){
            if(_id.length()){
                return _id.charAt(0);
            }
            return '\0';
        }
        
        String get_data(){
            return _data;
        }
        
        unsigned long get_hash_sum(){
        	unsigned long  sum = 0;
        	sum += (unsigned long)(_id.charAt(0));
            for(int i=0; i<_data.length(); i++){
            	sum += _data.charAt(i);
            }
            return sum;
        }

        bool get_boolean(){
        	return get_data().toInt();
        	/*
            if(get_data() == "0"){
                return false;
            }else{
                return true;
            }*/
        }
        
        String get_arg(int i){
            String arg;
            int _i = 0;
            for(int j=0; j<_data.length(); j++){
                if(_data.charAt(j) == ' '){
                    _i += 1;
                }else if(_i == i){
                    arg += String(_data.charAt(j));
                }else if(_i > i){
                    break;
                }
            }
            return arg;
        }
        
};

class SerialPort{

    private:
    
        long _speed = 57600;
        
        String _pending;
        
        bool _enabled = false;
    
    public:

        SerialPort(){
        }

        SerialPort(long speed){
            _speed = speed;
        }
        
        void init(){
            Serial.begin(_speed);
            enable();
        }
        
        void enable(){
            if(!_enabled){
                _enabled = true;
            }
        }
        
        void disable(){
            if(_enabled){
                //Serial.end();
                _enabled = false;
            }
        }
        
        Packet read(){
        
            String id = BLANK_ID;
            String data = String("");
            int i = 0;
        
            if(_enabled){
                while (Serial.available() > 0)
                {
                    char recieved = Serial.read();
            
                    // Process message when new line character is recieved
                    if(recieved == '\n'){
                        i = _pending.indexOf(' ');
                        if(i >= 0){
                            id = _pending.substring(0, i);
                            id.trim();
                            data = _pending.substring(i);
                            data.trim();
                        }else{
                            id = _pending;
                            id.trim();
                        }
                        _pending = ""; // Clear recieved buffer
                    }else{
                        _pending += recieved; 
                    }
                }
            }

            // Minimizes non-deterministic serial dropout?
            // Stops non-deterministic serial dropout?
            // This prevents opening on serial port on most recent bootloader?
//            Serial.end();
//            init();

            return Packet(id, data);
        }

        void write(String data){
        	// Note, the maximum bytes able to be written is define by SERIAL_BUFFER_SIZE,
        	// which by default is 64.
            if(_enabled && data.length()){
                Serial.println(data);
                Serial.flush();

                // Minimizes non-deterministic serial dropout?
                // This prevents opening on serial port on most recent bootloader?
//                Serial.end();
//                init();
            }
        }

        void log(String data){
        	write(String(ID_LOG)+String(' ')+String(millis())+String(' ')+data);
        }

};

#endif
