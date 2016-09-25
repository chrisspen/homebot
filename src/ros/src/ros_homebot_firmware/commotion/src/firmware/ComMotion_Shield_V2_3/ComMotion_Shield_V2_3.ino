
#include <EEPROM.h>
#include <Wire.h>
#include "IOpins.h"
#include "Commands.h"

byte mcu;                                                                     // designation as MCU 1 or MCU 2 as determined by state of pin A2
byte address;                                                                 // I²C address based on dipswitches and ID pin

//============================================================================== Motor Control Variables ================================================================================

int acount, bcount;                                                   // encoder pulse counters used to measure distance
//uint16_t acount_abs = 0, bcount_abs = 0;  // encoder pulse counters absolute, don't maintain directionality
volatile byte aflag;                                                          // flag to indicate encoder A has changed state                                                                
volatile byte bflag;                                                          // flag to indicate encoder B has changed state
volatile unsigned long apulse,bpulse;                                         // width of encoder pulses in uS
volatile unsigned long atime,btime;                                           // stores time of last encoder state change
byte motora,motorb;                                                           // values will be 0&1 on MCU1 and 2&3 on MCU2     - precalculated to increase speed
long maxpulse[4];                                                             // max time between encoder state changes in uS   - precalculated to increase speed

//============================================================================== Configuration Data======================================================================================

byte demo;                                                                    // demo mode                   modes : 0 = OFF,  1 = Line,  2 = Object
byte mode=0;                                                                  // default to standby          modes : 0 = standby, 1 = I²C command, 2 = Serial command, 3 = Scamper Demo
byte configuration=0;                                                         // default to Omni 3 wheel     config: 0 = Omni 3 wheel, 1 = Omni 4 wheel, 2 = Mecanum, 3 = Individual
byte lowbat=60;                                                               // low battery voltage (60 = 6.0V)
int maxamps[4];                                                               // used to set maximum current allowed (255 = 2.55A) for each motor
byte master=1;                                                                // external master address (default=1)
byte addroffset=0;                                                            // optional I²C address offset (default=0)
byte defaulted;                                                               // used to determine if defaults need to be loaded
byte encoders;                                                                // flag to indicate if encoder feedback is enabled or disabled
int motrpm[4];                                                                // motor rpm used to determine max speed
int encres[4];                                                                // encoder resolution (x100) used to determine max speed
byte reserve[4];                                                              // reserve power to ensure accurate speed under changing load
long stalltm[4];                                                              // stall time in mS used to determine if a motor is running slow or stalled
byte sermode;                                                                 // serial mode determines where commands come from and data goes to
unsigned int baudrate[2];                                                     // baud rate for each serial port


//============================================================================== Shield Control Variables ===============================================================================

int velocity=0;                                                               // requested speed
int angle=-360;                                                               // requested angle of travel
int rotation=0;                                                               // requested rotation
int mspeed[4] = {0, 0, 0, 0};                                                 // requested speed of individual motors
int mdir[4] = {0, 0, 0, 0};	      										      // motor direction, +1=>speed>0, 0=>speed==0 -1=>speed<0

float radconvert=PI/180;                                                      // used to convert radians into degrees (precalculated to improve speed)

byte analogpin[5]={0,1,3,6,7};                                                // analog inputs to scan: A0=current A, A1=current B, A3 & A6=spare, A7=voltage on MCU1 or spare / sensor on MCU2
int  analogvalue[5];                                                          // store analog input results
byte datapack[32];                                                            // command/config data packet
byte sendpack[32];                                                            // data send packet for returning status data
byte serpack[36];                                                             // serial data pack
byte syncpack=1;                                                              // sync pack type (default: 1=controller config) 
byte packsize;                                                                // size of command packet    
byte command = CMD_NULL;                                                      // 255 = no command 
byte analog;                                                                  // read different analog input each loop (analog conversion takes 260uS)
byte powerdown=0;                                                             // a value of 1 shuts down all motors to conserve power and prevent a brownout condition

int  voltage;                                                                 // battery voltage
byte eflag=0;                                                                 // error flag records faults such as over current and under voltage
byte i2cfreq=0;                                                               // default value for I²C clock: 0=100kHz, 1=400kHz

//============================================================================== Demo global variables ==================================================================================

unsigned long time=millis();
unsigned long IRtime=millis();

void PowerDown()
{
  analogWrite(pwmapin,0);                                                                       // power down M1
  analogWrite(pwmbpin,0);                                                                       // power down M2
  eflag=eflag|B00100000;                                                                        // error flag bit 6 indicates power shut down due to low battery voltage
  
  Wire.beginTransmission(address+1);                                                             // take control of I²C bus and address MCU2
  Wire.write(3);                                                                                 // datapacket #3 - send power down command to M3 & M4
  for(byte i=0;i<8;i++)                                                                       
  {
    Wire.write(0);                                                                                // set all motor speeds to 0
  }
  Wire.endTransmission();                                                                         // transmission ok - release control of I²C bus
}

void setup()
{
  //============================================================================ Use Reset Button to select demo mode ===================================================================
  
  EEPROMload();                                                               // load configuration from EEPROM
  if(defaulted!=170) EEPROMdefaults();                                        // load defaults if no previous configuration found or in demo mode
  
  for(byte i=0;i<4;i++)
  {
    maxpulse[i]=60000000L/(long(motrpm[i])*long(encres[i])/100L)*255L;
    maxpulse[i]=maxpulse[i]*(100L-long(reserve[i]))/100L;
  }
  
  DDRD=B00000011;                                                             // ensure dipswitch pins (PD4-PD7) and encoder inputs (PD2,PD3) plus RX and TX are set to input
  PORTD=PORTD|B11111100;                                                      // enable pullup resistors on dipswitch pins (PD4-PD7) and encoder inputs (PD2,PD3)
  DDRB=DDRB|B00001111;                                                        // set motor control pins PB0 - PB3 (D8,D9,D10,D11) as output
  

  mcu=digitalRead(IDpin);                                                     // low = MCU 1    high = MCU 2
  address=((PIND&B11110000)>>3)+addroffset+mcu;                               // I²C address is selected by dip switches + offset + state of ID pin (MCU1 or MCU2).
  
  // On mcu=0, motora=0, motorb=1, on mce=1, motora=2, motorb=3
  motora=mcu*2;
  motorb=mcu*2+1;
  
  Wire.begin(address);                                                        // initialize I²C library and set slave address
  Wire.onReceive(I2C_Receive);                                                // define I²C slave receiver ISR
  Wire.onRequest(I2C_Send);                                                   // define I²C slave transmit ISR
//  Wire.setTimeout(1L);                                                        // sets a timeout of 1mS for I²C
  Wire.setTimeout(1000L);                                                        // sets a timeout of 1mS for I²C
  Wire.flush();
  delay(100);                                                                 // required to ensure both processors are initialized before inter-communications begins
  
  //http://playground.arduino.cc/Code/ATMELTWI
  //TWBR = ((CPUFREQ / 400000L) - 16) / 2; // Set I2C frequency to 400kHz
  //TWBR = ((CPU_FREQ / 100000L) - 16) / 2; // Set I2C frequency to 100kHz
  //if(i2cfreq==0)                                                              // thanks to Nick Gammon: http://gammon.com.au/i2c
  //{
    //TWBR=72;                                                                  // default I²C clock is 100kHz
    Wire.setClock(100000L);
  //}else{
    //TWBR=12;                                                                  // change the I²C clock to 400kHz
  //}
  
  //if(mode==1 && mcu==0)                                                       // if demo mode is selected
  //{
    //demo+=1;                                                                  // toggle demo every time power is turned on or reset is pressed
    //if(demo>1) demo=0;                                                        // limit demo to 0, 1 or 2
    
    //angle=0;
    
    //EEPROM.write(0,demo);                                                     // update demo mode
    
    //Wire.beginTransmission(address+1);
    //datapack[0]=15;                                                           // command 15 used to syncronize demo mode 
    //datapack[1]=demo;
    //datapack[2]=mode;
    //Wire.write(datapack,3);
    //Wire.endTransmission();
  //}
  //else
  //{
    delay(10);
  //}

  //Serial.begin(long(baudrate[mcu]));                                          // initialize Serial library and set baud rate
  //Serial.setTimeout(1L);                                                      // sets a timeout of 1mS for Serial. Baud below 9600 should not be used unless this value is increased
  
  delay(1);
  
  while(millis()-time<1000)
  {
  }
  
  
  //============================================================================ Encoder Interrupts =====================================================================================

  attachInterrupt(0, Aencoder, CHANGE);                                         // call ISR for left  encoder when state changes
  attachInterrupt(1, Bencoder, CHANGE);                                         // call ISR for right encoder when state changes
}


void loop()
{
  
  // Disable IC1
  if(mcu == 0){
      delay(1000);
      return;
  }
  
  //---------------------------------------------------------------------------- Read analog inputs including battery voltage and motor currents ----------------------------------------
  analog++;                                                                   // select a different input each loop (analog read takes 260uS)
  if(analog>4) analog=0;                                                      // rotate through current A, current B, A3, A6 and A7
  
  analogvalue[analog] = analogRead(analogpin[analog]);                        // read selected analog input and store in array for later retrieval
  if(mcu==0 && analog==4) voltage=analogvalue[4]*30/185;                      // convert to battery voltage (60 = 6.0V)
  
  //---------------------------------------------------------------------------- Shut down motors if battery is equal or below lowbat ---------------------------------------------------

  //if(mcu==0 && analog==4 && powerdown<250)                                    // battery voltage has just been read and no powerdown has occured
  //{
  //  if(analogvalue[4]<=lowbat)                                                // compare battery voltage to low battery voltage
  //  {
  //    eflag=eflag|B00010000;                                                  // bit 5 indicates power dipping below batlow voltage
  //    powerdown++;                                                            // increment shutdown counter if battery voltage is low
  //  }
  //  else
  //  {
  //    powerdown=0;                                                            // reset shutdown counter if battery voltage is high
  //  }
  //  if(powerdown>249) PowerDown();                                            // if battery voltage consistantly low for 250 samples shutdown all motors
  //}
  ///if(powerdown>249) return;                                                   // power must be cycled or reset button pressed to resume
  
  //---------------------------------------------------------------------------- Shield Functions ---------------------------------------------------------------------------------------
  
  if(command<32) Commands();                                                  // respond to command from I²C bus or Serial interface
  
  Motors();                                                                 // if encoders are enabled then use then to control motor speeds
  
}


