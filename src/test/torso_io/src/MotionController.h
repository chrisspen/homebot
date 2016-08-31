
#include <Wire.h>
#include "Arduino.h"

#include "i2c_addresses.h"

// ComMotion variables
//#define COMMOTION_ADDR  30 // ComMotion Shield I²C address with all dip switches off
#define COMMOTION_OFFS   0 // ComMotion Shield I²C address offset (only needed if other I²C devices use addresses 0-31)

byte MCU=0;     // determines which MCU to request data from
byte datapack[32];         // data received from the ComMotion shield - I²C buffer is 32 bytes
byte request=B00001101;    // status request = encoder counts + motor currents + battery voltage
unsigned long stime;       // timer used so status updates do not flood I2C bus
int encoders[4];// encoder counts for motors 1-4 (negative values = reverse direction)
int mcurrent[4];// current draw   for motors 1-4 (205 = 1A)  
int analogin[6];// analog inputs from ComMotion shield MCU1 A3, MCU1 A6, Battery Voltage, MCU2 A3, MCU2 A6, MCU2 A7
byte errorlog;  // errorlog from ComMotion shield

// I²C Slave Receive Status 

void ReceiveStatus(int bytes) // empty I²C buffer into datapack
{
    Serial.println("Receiving!");
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
    
    Serial.println(((String)"Error: ") + errorlog);
  }
  
}

void BasicConfig(byte mode, byte chassis, byte lowbat, byte maxcur1, byte maxcur2, byte maxcur3, byte maxcur4, byte i2coffset, byte i2cmaster)
{
  Wire.beginTransmission(COMMOTION_ADDR+COMMOTION_OFFS);// Initialize I²C communications with ComMotion shield
  Wire.write(1);// Specify that data packet is basic configuration data
  Wire.write(mode);                 // 0=normal, 1=demo
  Wire.write(chassis);              // 0=3xOmni, 1=4xOmni, 2=Mecanum, 3=Individual. Add 16 to disable encoder feedback
  Wire.write(lowbat);               // 0-255    55=5.5V
  Wire.write(maxcur1);              // 0-255   255=2.55A
  Wire.write(maxcur2);              // 0-255   255=2.55A
  Wire.write(maxcur3);              // 0-255   255=2.55A
  Wire.write(maxcur4);              // 0-255   255=2.55A
  Wire.write(i2coffset);            // 0-95    a value of 32 changes ComMotion dipswitch address range to 32-63
  Wire.write(i2cmaster);            // I²C address for I²C master - used for emergency reports such as motor overload
  Wire.endTransmission();           // transmit data from I²C buffer to ComMotion shield
}

// Encoder Configuration, only need to send once
void EncoderConfig(int maxrpm, int encres, byte reserve, byte maxstall)
{
  Wire.beginTransmission(COMMOTION_ADDR+COMMOTION_OFFS); // Initialize I²C communications with ComMotion shield
  Wire.write(2);   // Specify that data packet is encoder configuration data
  Wire.write(highByte(maxrpm));       // high byte of motor RPM - 13500rpm for Scamper
  Wire.write( lowByte(maxrpm));       //  low byte of motor RPM - 8500rpm for Rover 5
  Wire.write(highByte(encres));       // high byte of encoder resolution x100 - 800 for Scamper
  Wire.write( lowByte(encres));       //  low byte of encoder resolution x100 - 200 for Rover 5
  Wire.write(reserve);                // 0-50%     reserve power - use when constant speed under variable load is critical
  Wire.write(maxstall);               // 1-255μS   number of μS between encoder pulses before stall is assumed  10 for Scamper, 25 for Rover 5
  Wire.endTransmission();             // transmit data from I²C buffer to ComMotion shield
}

void SerialConfig(int baud1, int baud2, byte smode)
{
  Wire.beginTransmission(COMMOTION_ADDR+COMMOTION_OFFS);                    // Initialize I²C communications with ComMotion shield
  Wire.write(4);                             // Specify that data packet is serial configuration data
  Wire.write(highByte(baud1));               // Baud rate for ComMotion serial port 1
  Wire.write( lowByte(baud1));               // Common baud rates are: 1200,2400,4800,9600,14400,28800,38400,57600,115200
  Wire.write(highByte(baud2));               // Baud rate for ComMotion serial port 2 (Xbee / WiFly)
  Wire.write( lowByte(baud2));               // Common baud rates are: 1200,2400,4800,9600,14400,28800,38400,57600,115200
  Wire.write(smode);                         // 0-4  Determines how ComMotion shield handles incoming data (default=4)
  Wire.endTransmission();                    // transmit data from I²C buffer to ComMotion shield
}

void ComMotionStatus(byte mcu, byte request) // value for mcu must be 0 or 1. Each bit of the request byte is a separate request.
{
  Wire.beginTransmission(COMMOTION_ADDR + COMMOTION_OFFS + mcu);            // Initialize I²C communications with ComMotion shield MCU 1 or 2
  Wire.write(6);                             // Specify that data packet is a status request
  Wire.write(request);                       // Each bit of the request byte returns a different status report
  Wire.endTransmission();                    // transmit data from I²C buffer to ComMotion shield
}
//---------------------------------------------------------------------------- Individual Motor Control ------------------------------------------------------------------------------------

void IndividualMotorControl(int m1, int m2, int m3, int m4)
{
  Wire.beginTransmission(COMMOTION_ADDR+COMMOTION_OFFS);                    // Initialize I²C communications with ComMotion shield
  Wire.write(3);                                                            // Specify that data packet is motor control data
  Wire.write(highByte(m1));                                                 // -255 to +255  speed for motor 1
  Wire.write( lowByte(m1));                                                 // negative values reverse direction of travel
  Wire.write(highByte(m2));                                                 // -255 to +255  speed for motor 2
  Wire.write( lowByte(m2));                                                 // negative values reverse direction of travel
  Wire.write(highByte(m3));                                                 // -255 to +255  speed for motor 3
  Wire.write( lowByte(m3));                                                 // negative values reverse direction of travel
  Wire.write(highByte(m4));                                                 // -255 to +255  speed for motor 4
  Wire.write( lowByte(m4));                                                 // negative values reverse direction of travel
  Wire.endTransmission();                                                   // transmit data from I²C buffer to ComMotion shield
}

class MotorController
{
    private:
    
        int last_speed;
        
        int target_speed;
        
        unsigned long target_last_set;
        
        unsigned int ramp_duration;
    
    public:
    
        MotorController(){
            last_speed = 0;
            target_speed = 0;
            target_last_set = millis();
            ramp_duration = 1000; // ms to transition
        }
        
        void set_ramp(unsigned int r){
            ramp_duration = r;
        }
        
        void set_speed(int s){
            last_speed = target_speed;
            target_speed = s;
            target_last_set = millis();
        }
        
        /*
        Get the speed incrementally set over a period of time to reach the target speed.
        */
        int get_speed(){
            unsigned long current_time = millis();
            int time_spent = current_time - target_last_set;
            float time_spent_ratio = (float)time_spent/(float)ramp_duration;
            int ret = 0;
            if(time_spent >= ramp_duration){
                ret = target_speed;
            }else{
                ret = (int)((float)(target_speed - last_speed)*time_spent_ratio + last_speed);
            }
            
            //Serial.println(((String)"speed: ") + ret);
            return ret;
        }
        
        /*
        Simply returns the target speed, even if the ramp duration has not been met.
        */
        int get_speed_instantly(){
            return target_speed;
        }
        
};

class MotionController
{

    private:
            
        MotorController _motor_left;
        MotorController _motor_right;

    public:
    
        MotionController(){
                    
            _motor_left = MotorController();
            _motor_right = MotorController();
            
            Wire.onReceive(ReceiveStatus);
            
            // Normal mode, Rover 5 with mecanum wheels, lowbat = 6V, motor currents =2.5A, no offset, Master address=1;
            BasicConfig(
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
            EncoderConfig(
                8500, //maxrpm=maximum rpms
                200, //encres=encoder resolution
                //600, //maxrpm=maximum rpms, Pololu Metal Gearmotor 2282
                //465, //encres=encoder resolution, Pololu Metal Gearmotor 2282
                10, //reserve=0-50%     reserve power - use when constant speed under variable load is critical
                25 //maxstall=1-255μS   number of μS between encoder pulses before stall is assumed  10 for Scamper, 25 for Rover 5
            );
            
            //TODO:fix
            // Set shield port 1 to 9600 baud, Set shield port 2 to 9600 baud, Pass all data back to I²C master for processing
            //SerialConfig(9600, 9600, 0);
            
        }
        
        void stop(){
            
            _motor_left.set_speed(0);
            _motor_right.set_speed(0);
            //NA NA left right
            IndividualMotorControl(0, 0, _motor_left.get_speed_instantly(), _motor_right.get_speed_instantly());
            
        }
};
