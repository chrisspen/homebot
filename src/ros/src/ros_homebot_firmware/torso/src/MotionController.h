
#include "Wire.h"
#include "Arduino.h"

#include "I2CAddresses.h"
#include "Sensor.h"

#define FULL_SPEED      255
#define HALF_SPEED      128
#define QUARTER_SPEED   64

#define COMMOTION_REQUEST_ENCODER B00000001
#define COMMOTION_REQUEST_CURRENT B00000100
#define COMMOTION_REQUEST_BATTERY B00001000
#define COMMOTION_REQUEST_ANALOG2 B00010000
#define COMMOTION_REQUEST_ERRORLOG B00100000
#define COMMOTION_STATUS_UPDATE_FREQ 50 // milliseconds

// ComMotion variables

byte MCU = 0;                     // determines which MCU to request data from
//byte datapack[32];              // data received from the ComMotion shield - I2C buffer is 32 bytes
//byte request;// = B00001101;         // status request = encoder counts + motor currents + battery voltage
//unsigned long stime;            // timer used so status updates do not flood I2C bus
//int encoders[4];                // encoder counts for motors 1-4 (negative values = reverse direction)
//int mcurrent[4];                // current draw   for motors 1-4 (205 = 1A)
//int analogin[6];                // analog inputs from ComMotion shield MCU1 A3, MCU1 A6, Battery Voltage, MCU2 A3, MCU2 A6, MCU2 A7
//byte errorlog;                  // errorlog from ComMotion shield

//unsigned long received_status_count = 0;
//unsigned long requested_status_count = 0;

// I2C Slave Receive Status 

/*
void handle_commotion_status_request()
{
    received_status_count += 1;
}
*/
/*
void handle_commotion_status_response(int bytes) // empty I2C buffer into datapack
{

    received_status_count += 1;

    for(int i=0;i<bytes;i++)
    {
        datapack[i]=Wire.read();
    }

    byte index=0;    // start at the beginning - counts number of bytes processed

    if(request&B00000001)       // request encoder counts - used to measure distance travelled 
    {
        encoders[MCU*2]=datapack[index]*256+datapack[index+1];
        encoders[MCU*2+1]=datapack[index+2]*256+datapack[index+3];    
        index+=4;
    }

    if(request&B00000100)       // request motor currents - can be used to determine if the robot has stalled
    {
        mcurrent[MCU*2]=datapack[index]*256+datapack[index+1];
        mcurrent[MCU*2+1]=datapack[index+2]*256+datapack[index+3];
        index+=4;
    }

    if(request&B00001000)       // read ComMotion shield analog inputs for MCU 1 including the battery voltage monitor
    {
        analogin[0]=datapack[index]*256+datapack[index+1];
        analogin[1]=datapack[index+2]*256+datapack[index+3];
        analogin[2]=datapack[index+4]*256+datapack[index+5];
        index+=6;
    }

    if(request&B00010000)       // read ComMotion shield analog inputs for MCU 2 
    {
        analogin[3]=datapack[index]*256+datapack[index+1];
        analogin[4]=datapack[index+2]*256+datapack[index+3];
        analogin[5]=datapack[index+4]*256+datapack[index+5];
        index+=6;
    }

    if(request&B00100000)       // request the error log
    {
        errorlog=datapack[index];
    }
  
}
*/

void set_basic_config(byte mode, byte chassis, byte lowbat, byte maxcur1, byte maxcur2, byte maxcur3, byte maxcur4, byte i2coffset, byte i2cmaster)
{
//Serial.println(String("A: connecting to ")+String(COMMOTION_ADDR));Serial.flush();
    Wire.beginTransmission(COMMOTION_ADDR);// Initialize I2C communications with ComMotion shield
//Serial.println(String("B"));Serial.flush();
    Wire.write(1);// Specify that data packet is basic configuration data
//Serial.println(String("C"));Serial.flush();
    Wire.write(mode);                 // 0=normal, 1=demo
//Serial.println(String("D"));Serial.flush();
    Wire.write(chassis);              // 0=3xOmni, 1=4xOmni, 2=Mecanum, 3=Individual. Add 16 to disable encoder feedback
//Serial.println(String("E"));Serial.flush();
    Wire.write(lowbat);               // 0-255    55=5.5V
//Serial.println(String("F"));Serial.flush();
    Wire.write(maxcur1);              // 0-255   255=2.55A
    Wire.write(maxcur2);              // 0-255   255=2.55A
    Wire.write(maxcur3);              // 0-255   255=2.55A
    Wire.write(maxcur4);              // 0-255   255=2.55A
    Wire.write(i2coffset);            // 0-95    a value of 32 changes ComMotion dipswitch address range to 32-63
    Wire.write(i2cmaster);            // I2C address for I2C master - used for emergency reports such as motor overload
//Serial.println(String("F2"));Serial.flush();
    Wire.endTransmission();           // transmit data from I2C buffer to ComMotion shield
//Serial.println(String("G"));Serial.flush();
}

// Encoder Configuration, only need to send once
void set_encoder_config(int maxrpm, int encres, byte reserve, byte maxstall)
{
    Wire.beginTransmission(COMMOTION_ADDR); // Initialize I2C communications with ComMotion shield
    Wire.write(2);   // Specify that data packet is encoder configuration data
    Wire.write(highByte(maxrpm));       // high byte of motor RPM - 13500rpm for Scamper
    Wire.write( lowByte(maxrpm));       //  low byte of motor RPM - 8500rpm for Rover 5
    Wire.write(highByte(encres));       // high byte of encoder resolution x100 - 800 for Scamper
    Wire.write( lowByte(encres));       //  low byte of encoder resolution x100 - 200 for Rover 5
    Wire.write(reserve);                // 0-50%     reserve power - use when constant speed under variable load is critical
    Wire.write(maxstall);               // 1-255������S   number of ������S between encoder pulses before stall is assumed  10 for Scamper, 25 for Rover 5
    Wire.endTransmission();             // transmit data from I2C buffer to ComMotion shield
}

void set_serial_config(int baud1, int baud2, byte smode)
{
    Wire.beginTransmission(COMMOTION_ADDR);                    // Initialize I2C communications with ComMotion shield
    Wire.write(4);                             // Specify that data packet is serial configuration data
    Wire.write(highByte(baud1));               // Baud rate for ComMotion serial port 1
    Wire.write( lowByte(baud1));               // Common baud rates are: 1200,2400,4800,9600,14400,28800,38400,57600,115200
    Wire.write(highByte(baud2));               // Baud rate for ComMotion serial port 2 (Xbee / WiFly)
    Wire.write( lowByte(baud2));               // Common baud rates are: 1200,2400,4800,9600,14400,28800,38400,57600,115200
    Wire.write(smode);                         // 0-4  Determines how ComMotion shield handles incoming data (default=4)
    Wire.endTransmission();                    // transmit data from I2C buffer to ComMotion shield
}
/*
void request_commotion_status(byte mcu, byte req) // value for mcu must be 0 or 1. Each bit of the request byte is a separate request.
{
    request = req;
    requested_status_count += 1;
    Wire.beginTransmission(COMMOTION_ADDR + mcu);            // Initialize I2C communications with ComMotion shield MCU 1 or 2
    Wire.write(6);                             // Specify that data packet is a status request
    Wire.write(req);                       // Each bit of the request byte returns a different status report
    Wire.endTransmission();                    // transmit data from I2C buffer to ComMotion shield
}
*/
//---------------------------------------------------------------------------- Individual Motor Control ------------------------------------------------------------------------------------

void set_motor_speeds(int m1, int m2, int m3, int m4)
{
    Wire.beginTransmission(COMMOTION_ADDR);                    // Initialize I2C communications with ComMotion shield
    Wire.write(3);                                                            // Specify that data packet is motor control data
    Wire.write(highByte(m1));                                                 // -255 to +255  speed for motor 1
    Wire.write( lowByte(m1));                                                 // negative values reverse direction of travel
    Wire.write(highByte(m2));                                                 // -255 to +255  speed for motor 2
    Wire.write( lowByte(m2));                                                 // negative values reverse direction of travel
    Wire.write(highByte(m3));                                                 // -255 to +255  speed for motor 3
    Wire.write( lowByte(m3));                                                 // negative values reverse direction of travel
    Wire.write(highByte(m4));                                                 // -255 to +255  speed for motor 4
    Wire.write( lowByte(m4));                                                 // negative values reverse direction of travel
    Wire.endTransmission();                                                   // transmit data from I2C buffer to ComMotion shield
}

class SpeedController
{
    private:
    
        // The actual speed we're telling the motor.
        // This takes into account acceleration.
        int current_speed;
        
        // The desired speed.
        int target_speed;
        
        // The time when the target speed was last set.
        // Used to time acceleration.
        unsigned long target_last_set;
        
        // The amount of velocity to add every second.
        // Pro-rated to milliseconds.
        unsigned int acceleration = 128; // takes 2 seconds to go from 0 to FULL_SPEED
        
        // timer used so status updates do not flood I2C bus
        unsigned long stime;
    
    public:
    
        SpeedController(){
            current_speed = 0;
            target_speed = 0;
            target_last_set = millis();
        }
        
        void set_speed(int s){
            if(s > FULL_SPEED){
                s = FULL_SPEED;
            }else if(s < -FULL_SPEED){
                s = -FULL_SPEED;
            }

            // If we haven't reached target before change, then save partial target
            // as a start for the new calculation.
            if(target_speed != current_speed){
            	current_speed = get_speed();
            }

            target_speed = s;
            target_last_set = millis();
        }
        
        void set_acceleration(unsigned int a){
            acceleration = a;
        }
        
        unsigned int get_acceleration(){
            return acceleration;
        }

        // Get the speed incrementally set over a period of time to reach the target speed.
        int get_speed(){
            int ret_speed = current_speed;
            unsigned long time_since_set = millis() - target_last_set;
            int speed_difference = abs(target_speed - current_speed);
            if(speed_difference){

            	// Calculate the maximum amount of time it should take to accelerate
            	// to the target speed.
                unsigned long time_to_change = float(speed_difference)/float(acceleration)*1000;

                if(time_since_set >= time_to_change){
                    // After acceleration was calculated to reach the target speed,
                    // stop updating and just set the value.
                    current_speed = target_speed;
                    ret_speed = current_speed;
                }else{
                    // Otherwise, calculate the gradual speed change.
                	// We don't save this change and instead calculate it from the original
                	// current_speed so as to not introduce rounding errors.
                	// Once we achieve target speed, we'll save the value, and this function will
                	// then become a glorified getattr around current_speed.
                    int polarity = (target_speed > current_speed) ? +1 : -1;
                    int vel_change = time_since_set * (1/1000.) * acceleration * polarity;
                    if(polarity > 0){
                        ret_speed = min(current_speed + vel_change, target_speed);
                    }else{
                        ret_speed = max(current_speed + vel_change, target_speed);
                    }
                }
            }
            return ret_speed;
        }
        
        // Simply returns the target speed, even if we've not yet fully accelerated.
        int get_speed_instantly(){
            current_speed = target_speed;
            return target_speed;
        }
        
};

class MotionController: public Sensor
{

    private:
            
        SpeedController _motor_left;
        SpeedController _motor_right;
        
        bool _connected = false;

    public:
    
        MotionController(){
        }
        
        bool is_connected(){
            return _connected;
        }
        
        void init(){
                  
            _motor_left = SpeedController();
            _motor_right = SpeedController();
            
            // Do this in main.ino setup()
            //Wire.onReceive(handle_commotion_status_response);
            
            Wire.beginTransmission(COMMOTION_ADDR);
            int error = Wire.endTransmission();
            if(!error){
                _connected = true;
            }else{
                return;
            }
            
            // Normal mode, Rover 5 with mecanum wheels, lowbat = 6V, motor currents =2.5A, no offset, Master address=1;
            //Serial.println(String(F("set_basic_config()"));Serial.flush();
            set_basic_config(
                0, //mode, 0=normal
                3, //chassis, 3=individual
                60, //lowbat, 0-255    55=5.5V (given 6V battery, 80%=4.8V=dead, 92%=5.5V=low)
                250, //maxcur1, 0-255   255=2.55A (left)
                250, //maxcur2, 0-255   255=2.55A (right)
                250, //maxcur3, 0-255   255=2.55A (unused)
                250, //maxcur4, 0-255   255=2.55A (unused)
                0, //i2coffset
                1 //i2cmaster
            );
            
            // Max motor rpm = 8500rpm, encoder resolution = 2.00 state changes per motor revolution, 10% reserve power, stall at 25uS
            //Serial.println(String(F("set_encoder_config()")));Serial.flush();
            set_encoder_config(
                8500, //maxrpm=maximum rpms
                200, //encres=encoder resolution
                //600, //maxrpm=maximum rpms, Pololu Metal Gearmotor 2282
                //465, //encres=encoder resolution, Pololu Metal Gearmotor 2282
                10, //reserve=0-50%     reserve power - use when constant speed under variable load is critical
                25 //maxstall=1-255mS   number of ������S between encoder pulses before stall is assumed  10 for Scamper, 25 for Rover 5
            );
            
            //TODO:fix
            // Set shield port 1 to 9600 baud, Set shield port 2 to 9600 baud, Pass all data back to I2C master for processing
            //Serial.println(String(F("set_serial_config()"));Serial.flush();
            //set_serial_config(9600, 9600, 0);//TODO:fix? corrupts Arduino's serial?
            
            //Serial.println(String(F("motor.init() done"));Serial.flush();
            
        }
        
        void set(int left_speed, int right_speed){
            if(!is_connected()){
                return;
            }
            _motor_left.set_speed(left_speed);
            _motor_right.set_speed(right_speed);
            //NA NA left right
            set_motor_speeds(0, 0, _motor_left.get_speed(), _motor_right.get_speed());
        }
        
        void set_acceleration(unsigned int a){
            _motor_left.set_acceleration(a);
            _motor_right.set_acceleration(a);
        }

        unsigned int get_acceleration(){
            return _motor_left.get_acceleration();
        }

        virtual void update(){
            if(!is_connected()){
                return;
            }
            
            set_motor_speeds(0, 0, _motor_left.get_speed(), _motor_right.get_speed());
            
            /*
            // Query encoder state.
            // 2016.2.12 CKS This doesn't work.
            // Using MCU=0 it doesn't seem to break anything, but no I2C response is ever recieved.
            // Using MCU=1 causes it to hang indefinitely.
            // request status update from ComMotion shield every 50mS (20 times a second)
            if(millis()-stime >= COMMOTION_STATUS_UPDATE_FREQ){
                // toggle between MCU 1 and MCU 2
                //MCU++;if(MCU>1) MCU=0;//causes arduino/commotion to crash/hang?
                //MCU=1;//causes arduino/commotion to crash/hang
                //MCU=0;
                stime = millis();
                //request_commotion_status(MCU, B00000001|B00100000);
                request_commotion_status(MCU, B00000001);
            }
            */
        }
        
        void stop(){
            if(!is_connected()){
                return;
            }
            
            _motor_left.set_speed(0);
            _motor_right.set_speed(0);
            //NA NA left right
            set_motor_speeds(0, 0, _motor_left.get_speed_instantly(), _motor_right.get_speed_instantly());
            
        }

        virtual bool get_and_clear_changed(){
        	return false;//TODO?
        }

        virtual String get_reading_packet(){
        	return String();//TODO?
        }

        String get_acceleration_packet(){
            return String(ID_GET_VALUE)+String(' ')+
                String(ID_MOTOR_ACCEL)+String(' ')+
                String(get_acceleration());
        }

};
