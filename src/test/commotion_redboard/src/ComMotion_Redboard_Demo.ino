
//                        ************************************************************************************************************************************************************
//                        *  This sample code contains all the commands used with the ComMotion shield in the ComMotion_Commands tab at the top of the screen.                       *
//                        *  Connect the output of an IR receiver to D2 to receive IR commands from a SONY TV remote or universal TV remote set to the SONY IR protocol.             *
//                        *                                                                                                                                                          *
//                        *  The configuration settings shown are for a Rover 5 chassis using mecanum wheels but can be easily changed to suit your needs. Refer to the manual.      *
//                        *  The configuration in the setup only needs to be sent once. The ComMotion shield stores all configuration data in it's EEPROMs.                          *
//                        *                                                                                                                                                          *
//                        *  It should be noted that the two processors on the ComMotion shield occasionally communicate with each other via the I2C bus.                            *
//                        *  If the I2C bus is being used to transfer large amounts of data or the master does not release the bus when finished then this may                       *
//                        *  interfer with the function of the ComMotion shield. Serial communications through the ComMotion Shield should also be limited to small amounts of data. *
//                        ************************************************************************************************************************************************************


#include <Wire.h>

#define IRpin 2                                                             // Input from IR receiver. This must be an external interrupt pin (D2 or D3 for most Arduinos).
#define COMMOTION_ADDR  30                                                  // ComMotion Shield I²C address with all dip switches off
#define COMMOTION_OFFS   0                                                  // ComMotion Shield I²C address offset (only needed if other I²C devices use addresses 0-31)

volatile byte IRC;                                                          // global variable used to store IR Command
byte MCU=0;                                                                 // determines which MCU to request data from
byte datapack[32];                                                          // data received from the ComMotion shield - I²C buffer is 32 bytes
byte request=B00001101;                                                     // status request = encoder counts + motor currents + battery voltage
unsigned long stime;                                                        // timer used so status updates do not flood I2C bus

int encoders[4];                                                            // encoder counts for motors 1-4 (negative values = reverse direction)
int mcurrent[4];                                                            // current draw   for motors 1-4 (205 = 1A)  
int analogin[6];                                                            // analog inputs from ComMotion shield MCU1 A3, MCU1 A6, Battery Voltage, MCU2 A3, MCU2 A6, MCU2 A7
byte errorlog;                                                              // errorlog from ComMotion shield

unsigned long receive_count = 0;

//------------------------------------------------------------------ I²C Slave Receive Status -----------------------------------------

void ReceiveStatus(int bytes)                                     // empty I²C buffer into datapack
{
  receive_count += 1;
  for(int i=0;i<bytes;i++)
  {
    datapack[i]=Wire.read();
  }

  byte index=0;                                                   // start at the beginning - counts number of bytes processed
  
  if(request&B00000001)                                           // request encoder counts - used to measure distance travelled 
  {
    encoders[MCU*2]=datapack[index]*256+datapack[index+1];
    encoders[MCU*2+1]=datapack[index+2]*256+datapack[index+3];    
    index+=4;
  }
  
  if(request&B00000100)                                           // request motor currents - can be used to determine if the robot has stalled
  {
    mcurrent[MCU*2]=datapack[index]*256+datapack[index+1];
    mcurrent[MCU*2+1]=datapack[index+2]*256+datapack[index+3];
    index+=4;
  }
  
  if(request&B00001000)                                           // read ComMotion shield analog inputs for MCU 1 including the battery voltage monitor
  {
    analogin[0]=datapack[index]*256+datapack[index+1];
    analogin[1]=datapack[index+2]*256+datapack[index+3];
    analogin[2]=datapack[index+4]*256+datapack[index+5];
    index+=6;
  }
  
  if(request&B00010000)                                           // read ComMotion shield analog inputs for MCU 2 
  {
    analogin[3]=datapack[index]*256+datapack[index+1];
    analogin[4]=datapack[index+2]*256+datapack[index+3];
    analogin[5]=datapack[index+4]*256+datapack[index+5];
    index+=6;
  }
  
  if(request&B00100000)                                           // request the error log
  {
    errorlog=datapack[index];
  }
}

void IRdecode()
{
  volatile static unsigned long irbitwidth;  // width of data bit in uS
  volatile static byte irdata;               // temporary storage of byte value as bits are read
  volatile static byte irbitvalue;           // decimal value of bit being read (1,2,4,8,16,32,64,128)
  volatile static byte irbitcount;           // counts number of bits since start bit (only first 7 bits are used)       

  if(digitalRead(IRpin)==0)                  // check state of IR receiver output 
  {
    irbitwidth=micros();                     // bit starts when IR receiver pin goes low
  }
  else                                       // IR receiver output high - end of bit
  {
    irbitwidth=micros()-irbitwidth;          // measure width of bit
    if(irbitwidth>800ul)                     // bitwidth>800uS = logic "1" (logit 1 typically 1200uS)
    {
      irdata+=irbitvalue;                    // increment data by bit value
    }
    irbitvalue*=2;                           // bitvalue updated for next bit (1,2,4,8,16,32,64,128)
    irbitcount++;                            // count the bits

    if(irbitcount==7)                        // only want first 7 bits
    {
      irbitvalue=0;                          // additional bits have a value of 0
      IRC=irdata+1;                          // make data available to user
    }

    if(irbitwidth>1800ul)                    // start bit resets decoder values (startbit typically 2400uS)
    {
      irbitvalue=1;
      irbitcount=0;
      irdata=0;
    }  
  }
}


//--------------------------------------------------------------------------- Basic Configuration, only nned to send once ------------------------------------------------------------------

void BasicConfig(byte mode, byte chassis, byte lowbat, byte maxcur1, byte maxcur2, byte maxcur3, byte maxcur4, byte i2coffset, byte i2cmaster)
{
  Wire.beginTransmission(COMMOTION_ADDR+COMMOTION_OFFS);                    // Initialize I²C communications with ComMotion shield
  Wire.write(1);                                                            // Specify that data packet is basic configuration data
  Wire.write(mode);                                                         // 0=normal, 1=demo
  Wire.write(chassis);                                                      // 0=3xOmni, 1=4xOmni, 2=Mecanum, 3=Individual. Add 16 to disable encoder feedback
  Wire.write(lowbat);                                                       // 0-255    55=5.5V
  Wire.write(maxcur1);                                                      // 0-255   255=2.55A
  Wire.write(maxcur2);                                                      // 0-255   255=2.55A
  Wire.write(maxcur3);                                                      // 0-255   255=2.55A
  Wire.write(maxcur4);                                                      // 0-255   255=2.55A
  Wire.write(i2coffset);                                                    // 0-95    a value of 32 changes ComMotion dipswitch address range to 32-63
  Wire.write(i2cmaster);                                                    // I²C address for I²C master - used for emergency reports such as motor overload
  Wire.endTransmission();                                                   // transmit data from I²C buffer to ComMotion shield
}



//---------------------------------------------------------------------------- Encoder Configuration, only need to send once ----------------------------------------------------------------

void EncoderConfig(int maxrpm, int encres, byte reserve, byte maxstall)
{
  Wire.beginTransmission(COMMOTION_ADDR+COMMOTION_OFFS);                    // Initialize I²C communications with ComMotion shield
  Wire.write(2);                                                            // Specify that data packet is encoder configuration data
  Wire.write(highByte(maxrpm));                                             // high byte of motor RPM - 13500rpm for Scamper
  Wire.write( lowByte(maxrpm));                                             //  low byte of motor RPM - 8500rpm for Rover 5
  Wire.write(highByte(encres));                                             // high byte of encoder resolution x100 - 800 for Scamper
  Wire.write( lowByte(encres));                                             //  low byte of encoder resolution x100 - 200 for Rover 5
  Wire.write(reserve);                                                      // 0-50%     reserve power - use when constant speed under variable load is critical
  Wire.write(maxstall);                                                     // 1-255μS   number of μS between encoder pulses before stall is assumed  10 for Scamper, 25 for Rover 5
  Wire.endTransmission();                                                   // transmit data from I²C buffer to ComMotion shield
}



//---------------------------------------------------------------------------- Motor Control for Omni and Mecanum wheels -------------------------------------------------------------------
  
void MotorControl(int velocity, int angle, int rotation)
{
  Wire.beginTransmission(COMMOTION_ADDR+COMMOTION_OFFS);                    // Initialize I²C communications with ComMotion shield
  Wire.write(3);                                                            // Specify that data packet is motor control data
  Wire.write(highByte(velocity));                                           // -255 to +255  velocity/direction for preconfigured chassis
  Wire.write( lowByte(velocity));                                           // negative values reverse direction of travel
  Wire.write(highByte(angle));                                              // -255 to +255  angle of travel for chassis using omni or mecanum wheels
  Wire.write( lowByte(angle));                                              // negative values reverse direction of travel by 180°
  Wire.write(highByte(rotation));                                           // -255 to +255  desired rotation speed
  Wire.write( lowByte(rotation));                                           // negative values reverse direction of rotation
  Wire.endTransmission();                                                   // transmit data from I²C buffer to ComMotion shield
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



//---------------------------------------------------------------------------- Serial Port Configuration, only need to send once if required ----------------------------------------------
  
void SerialConfig(int baud1, int baud2, byte smode)
{
  Wire.beginTransmission(COMMOTION_ADDR+COMMOTION_OFFS);                    // Initialize I²C communications with ComMotion shield
  Wire.write(4);                                                            // Specify that data packet is serial configuration data
  Wire.write(highByte(baud1));                                              // Baud rate for ComMotion serial port 1
  Wire.write( lowByte(baud1));                                              // Common baud rates are: 1200,2400,4800,9600,14400,28800,38400,57600,115200
  Wire.write(highByte(baud2));                                              // Baud rate for ComMotion serial port 2 (Xbee / WiFly)
  Wire.write( lowByte(baud2));                                              // Common baud rates are: 1200,2400,4800,9600,14400,28800,38400,57600,115200
  Wire.write(smode);                                                        // 0-4  Determines how ComMotion shield handles incoming data (default=4)
  Wire.endTransmission();                                                   // transmit data from I²C buffer to ComMotion shield
}



//---------------------------------------------------------------------------- Send Serial Data, destination depends on serial config -----------------------------------------------------

void SendSerialData(byte port, String sdata)
{
  if(sdata.length()>31) return;                                             // Serial data cannot be more than 31 bytes in length due to size of I²C buffer
  Wire.beginTransmission(COMMOTION_ADDR+COMMOTION_OFFS);                    // Initialize I²C communications with ComMotion shield
  Wire.write(5);                                                            // Specify that data packet is serial data to be sent
  for (byte i=0;i<sdata.length();i++)                                       // Scan through string 1 byte at a time (ignores trailing null charactor)
  {
    Wire.write(sdata.charAt(i));                                            // stores data in I²C buffer (maximum 31 charactors)
  }
  Wire.endTransmission();                                                   // transmit data from I²C buffer to ComMotion shield
}



//---------------------------------------------------------------------------- Status Request can be directed to MCU 1 or MCU 2 ----------------------------------------------------------

void ComMotionStatus(byte mcu, byte request)                                // value for mcu must be 0 or 1. Each bit of the request byte is a separate request.
{
  Wire.beginTransmission(COMMOTION_ADDR + COMMOTION_OFFS + mcu);            // Initialize I²C communications with ComMotion shield MCU 1 or 2
  Wire.write(6);                                                            // Specify that data packet is a status request
  Wire.write(request);                                                      // Each bit of the request byte returns a different status report
  Wire.endTransmission();                                                   // transmit data from I²C buffer to ComMotion shield
}




void setup()
{
	Serial.begin(57600); // must match ino.ini
	
	Wire.begin(1);                                                            // Join I²C bus as Master with address #1
	Wire.onReceive(ReceiveStatus);
  
	BasicConfig(0,2,60,250,250,250,250,0,1);                                  // Normal mode, Rover 5 with mecanum wheels, lowbat = 6V, motor currents =2.5A, no offset, Master address=1;
	EncoderConfig(8500,200,10,25);                                            // Max motor rpm = 8500rpm, encoder resolution = 2.00 state changes per motor revolution, 10% reserve power, stall at 25uS
	SerialConfig(9600, 9600, 0);                                              // Set shield port 1 to 9600 baud, Set shield port 2 to 9600 baud, Pass all data back to I²C master for processing
}



void loop()
{
  if(IRC!=0)                                                                // has a SONY IR command been received?                                               
  {
    Serial.println(IRC,DEC);                                                // display received command                
    delay(100);                                                             // most remotes will repeat signal 2 or 3 times, use this delay to prevent repetition of command                           
    IRC=0;                                                                  // reset IRC until new command received
  }
  
  //if(millis()-stime>49)// request status update from ComMotion shield every 50mS (20 times a second)
  if(millis()-stime>1000)
  {
	Serial.println("Requesting update..."); Serial.flush();
	Serial.println(String("receive_count:")+String(receive_count)); Serial.flush();
    stime=millis();                                                         // reset timer
    MCU++;if(MCU>1) MCU=0;                                                  // toggle between MCU 1 and MCU 2
    ComMotionStatus(MCU, request);                                          // request status data from ComMotion shield
    int volts=analogin[2]*30/185;
    Serial.print("Battery Voltage:");Serial.print(volts/10);Serial.print(".");Serial.println(volts%10);
    Serial.print("M1 current:");mcurrent[0];Serial.print("\tM2 current:");mcurrent[1];Serial.print("\tM3 current:");mcurrent[2];Serial.println("\tM4 current:");mcurrent[3];
    Serial.print("M1 encoder:");encoders[0];Serial.print("\tM2 encoder:");encoders[1];Serial.print("\tM3 encoder:");encoders[2];Serial.println("\tM4 encoder:");encoders[3];
    Serial.println("");
  }
}




  
  
