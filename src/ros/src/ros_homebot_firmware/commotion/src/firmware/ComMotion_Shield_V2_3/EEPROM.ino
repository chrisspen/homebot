void EEPROMdefaults()
{
  byte defaults[]=
  {1,1,0,60,255,255,255,255                
  ,1,0,0,52,188,52,188,52,188,52,188
  ,3,32,3,32,3,32,3,32
  ,10,10,10,10,10,10,10,10
  ,4,37,128,37,128,170};                                            // default configuration data
                  
  for(int i=0;i<41;i++)
  {
    EEPROM.write(i,defaults[i]);                                    // store defaultes in EEPROM
  }
  EEPROMload();                                                     // load configuration from EEPROM
}


void EEPROMload()                                                   // load configuration from EEPROM into program
{
  demo=EEPROM.read(0);                                              // demo mode
  mode=EEPROM.read(1);                                              // shield mode
  
  encoders=1;                                                       // default to encoders enabled
  configuration=EEPROM.read(2);                                     // configuration
  //Encoders are always used.
  //if(configuration&16) encoders=0;                                  // disable encoders
    
  lowbat=EEPROM.read(3);                                            // low battery
  for(byte i=0;i<4;i++)                                             // load motor current limits
  {
    maxamps[i]=EEPROM.read(4+i);                                    // maxamp values
    motrpm[i]=EEPROM.read(i*2+11)*256+EEPROM.read(i*2+12);          // motor RPM values
    encres[i]=EEPROM.read(i*2+19)*256+EEPROM.read(i*2+20);          // encoder resolution (x100)
    reserve[i]=EEPROM.read(27+i);                                   // motor power reserve values
    stalltm[i]=long(EEPROM.read(31+i));                             // motor stall time in mS
  }
  
  master=EEPROM.read(8);                                            // external I²C master address
  addroffset=EEPROM.read(9);                                        // shield I²C address offset
  //i2cfreq=EEPROM.read(10);                                          // I²C clock: 0=100kHz, 1=400kHz
  
  sermode=EEPROM.read(35);
  baudrate[0]=(EEPROM.read(36)*256U+EEPROM.read(37));
  baudrate[1]=(EEPROM.read(38)*256U+EEPROM.read(39));
  
  defaulted=EEPROM.read(40);                                        // flag to indicate defaults have been loaded
}


void EEPROMsave()                                                   // save configuration data from program to EEPROM
{
  EEPROM.write(0,demo);
  EEPROM.write(1,mode);
  EEPROM.write(2,configuration);
  EEPROM.write(3,lowbat);
  
  EEPROM.write(4,maxamps[0]);
  EEPROM.write(5,maxamps[1]);
  EEPROM.write(6,maxamps[2]);
  EEPROM.write(7,maxamps[3]);
  
  EEPROM.write(8,master);
  EEPROM.write(9,addroffset);
  EEPROM.write(10,i2cfreq);
  
  for(byte i=0;i<4;i++)
  {
    EEPROM.write(i*2+11,highByte(motrpm[i]));
    EEPROM.write(i*2+12, lowByte(motrpm[i]));
    EEPROM.write(i*2+19,highByte(encres[i]));
    EEPROM.write(i*2+20, lowByte(encres[i]));
    EEPROM.write(i+27,reserve[i]);
    EEPROM.write(i+31,byte(stalltm[i]));
  }
  EEPROM.write(35,sermode);
  EEPROM.write(36,highByte(baudrate[0]));
  EEPROM.write(37, lowByte(baudrate[0]));
  EEPROM.write(38,highByte(baudrate[1]));
  EEPROM.write(39, lowByte(baudrate[1]));
  
  EEPROM.write(40,170);
}

/*
    EEPROM MAP
    
    addr  description

    0     demo mode:       true=active    false=paused
    1     shield mode:     true=demo      false=I²C / Serial
    2     configuration:   0=3xomni       1=4xomni        2=mecanum        3=individual        +16 (bit 4 high)=no encoders
    3     low bat volts:   60=6.0V
    4     M1 current:      255=2.55A
    5     M2 current:      255=2.55A
    6     M3 current:      255=2.55A
    7     M4 current:      255=2.55A

    8     master I²C addr: address of external master (default=1)
    9     I²C addr offset: shield address offset (default=0)
    10    I²C Clock:       0=100  1=400
    11    M1 RPM Hi:       Motor 1 RPM high byte
    12    M1 RPM Lo:       Motor 1 RPM low  byte
    13    M2 RPM Hi:       Motor 2 RPM high byte
    14    M2 RPM Lo:       Motor 2 RPM low  byte
    15    M3 RPM Hi:       Motor 3 RPM high byte
    16    M3 RPM Lo:       Motor 3 RPM low  byte
    17    M4 RPM Hi:       Motor 4 RPM high byte
    18    M4 RPM Lo:       Motor 4 RPM low  byte

    19    E1 Res Hi:       Encoder 1 resolution high byte
    20    E1 Res Lo:       Encoder 1 resolution low  byte
    21    E2 Res Hi:       Encoder 2 resolution high byte
    22    E2 Res Lo:       Encoder 2 resolution low  byte
    23    E3 Res Hi:       Encoder 3 resolution high byte
    24    E3 Res Lo:       Encoder 3 resolution low  byte
    25    E4 Res Hi:       Encoder 4 resolution high byte
    26    E4 Res Lo:       Encoder 4 resolution low  byte

    27    M1 Reserve:      Motor 1 reserve power
    28    M2 Reserve:      Motor 2 reserve power
    29    M3 Reserve:      Motor 3 reserve power
    30    M4 Reserve:      Motor 4 reserve power
    31    M1 stall time:   Motor 1 stall time
    32    M2 stall time:   Motor 2 stall time
    33    M3 stall time:   Motor 3 stall time
    34    M4 stall time:   Motor 4 stall time
    
    35    Serial Mode:     0=pass all data to master    1=accept motor control from port 1    2-accept motor control from port 2
			   3=accept control from and return data to port 1	 4=accept control from and return data to port 2

    36    Baud 1 Hi:	   Port 1 baud rate high byte
    37	  Baud 1 Lo:       Port 1 baud rate low  byte
    38    Baud 2 Hi:	   Port 2 baud rate high byte
    39    Baud 2 Lo:       Port 2 baud rate low  byte
        
    40    defaults flag    170=defaults loaded (170 = B10101010)
*/

