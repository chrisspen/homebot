/*
Modified SerialPort.h, attempting to improve performance by using a fixed length character array instead of
a String for serial data handling.
*/
#ifndef SerialPort_h
#define SerialPort_h

#include "ID.h"

#define HEAD "HEAD"
#define TORSO "TORSO"

#define MOTOR_FORWARD "forward"
#define MOTOR_REVERSE "reverse"
#define MOTOR_TURN_CW "turn_cw"
#define MOTOR_TURN_CCW "turn_ccw"
#define MOTOR_BREAK "break"

#define OK "OK"
#define PONG "PONG"

#define WAITING_FOR_ID 1
#define WAITING_FOR_NL 2

#define MAX_ID_LENGTH 10
#define MAX_DATA_LENGTH 50

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
        	return _id != String("");
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
        
        unsigned int mode = WAITING_FOR_ID;

        char _id[MAX_ID_LENGTH];
        unsigned int _id_length = 0;

        char _data[MAX_DATA_LENGTH];
        unsigned int _data_length = 0;

        //String _pending;
        
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
        
            //String id = String("");
            //String data = String("");
            int i = 0;
        
            if(_enabled){
                while (Serial.available() > 0)
                {
                    char received = Serial.read();
                    if(mode == WAITING_FOR_ID){
                    	if(_id_length+2 >= MAX_ID_LENGTH){
                    		// Corrupt.
                    		_id_length = 0;
                    		_data_length = 0;
                    		break;
                    	}else if(received == ' '){
                    		mode = WAITING_FOR_NL;
                    	}else if(received == '\n'){
                    		mode = WAITING_FOR_ID;
                    		break;
                    	}else{
                    		_id[_id_length] = received;
                    		_id_length += 1;
                    	}
                    }else if(mode == WAITING_FOR_NL){
                    	if(_data_length+2 >= MAX_DATA_LENGTH){
							// Corrupt.
							_id_length = 0;
							_data_length = 0;
							break;
                    	}else if(received == '\n'){
                    		mode = WAITING_FOR_ID;
                    		break;
                    	}else{
                    		_data[_data_length] = received;
                    		_data_length += 1;
                    	}
                    }
                }
            }
            
            _id[_id_length] = '\0';
            _data[_data_length] = '\0';

            Packet packet = Packet(String(_id), String(_data));

            _id_length = 0;
            _data_length = 0;
            return packet;
        }

        void write(String data){
        	// Note, the maximum bytes able to be written is define by SERIAL_BUFFER_SIZE,
        	// which by default is 64.
            if(_enabled && data.length()){
                Serial.println(data);
                Serial.flush();
            }
        }

        void log(String data){
        	write(String(ID_LOG)+String(' ')+String(millis())+String(' ')+data);
        }

};

#endif
