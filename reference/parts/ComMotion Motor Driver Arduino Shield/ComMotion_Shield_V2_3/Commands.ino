
void Commands()
{
  if(mcu==0 && (command<5 || command==10))                                   // if this is MCU1 and command is 0-4 then repeat command to MCU2 
  {
    Wire.beginTransmission(address+1);                                       // take control of I²C bus and address MCU2
    Wire.write(datapack,packsize);                                           // relay commands to MCU2
    Wire.endTransmission();                                   
  }
  
  if(command==10) 
  {
    EEPROMdefaults();
    command=255;
    return;
  }
  
  if(command>15) command-=16;                                                // 16 is added (bit 3 set high) for internal commands (no repeat)
  
  
  
  
  if(command==1 && packsize==10) //============================================ Basic Configuration Data Received =================================================
  {
    mode=datapack[1];
    configuration=datapack[2];
    lowbat=datapack[3];
    maxamps[0]=datapack[4];
    maxamps[1]=datapack[5];
    maxamps[2]=datapack[6];
    maxamps[3]=datapack[7];
    addroffset=datapack[8];
    master=datapack[9];
    EEPROMsave();                                                            // update EEPROM
    TXconfig();
    command=255;
    return;
  }

  if(command==2) //============================================================ Encoder Configuration Data Received =================================================
  {
    if(packsize==25)                                                         // configure each encoder individually
    {
      for(byte i=0;i<4;i++)
      {
        motrpm[i]=datapack[i*2+1]*256+datapack[i*2+2];
        encres[i]=datapack[i*2+9]*256+datapack[i*2+10];
        reserve[i]=datapack[18+i];
        stalltm[i]=datapack[22+i];
      }
    }
    else if(packsize==7)                                                     // use 1 configuration for all encoders
    {
      for(byte i=0;i<4;i++)
      {
        motrpm[i]=datapack[1]*256+datapack[2];
        encres[i]=datapack[3]*256+datapack[4];
        reserve[i]=datapack[5];
        stalltm[i]=datapack[6];
      }
    }
    for(byte i=0;i<4;i++)
    {
      maxpulse[i]=60000000L/(long(motrpm[i])*long(encres[i])/100L)*255L;     // update maxpulse values
      maxpulse[i]=maxpulse[i]*(100L-long(reserve[i]))/100L;
    }
    EEPROMsave();                                                            // update EEPROM
    TXconfig();
    command=255;
    return;
  }

  if(command==3 && (packsize==7 || packsize==9)) //============================ Motor Control =======================================================================
  {
    if((configuration==3 || configuration==19) && packsize==9)               // Individual motor control  
    {
      for(byte i=0;i<4;i++)
      {
        mspeed[i]=datapack[i*2+1]*256+datapack[i*2+2];
      }
    }
    else                                                                     // Omni or Mecanum Wheels 
    {
      velocity=datapack[1]*256+datapack[2];
      angle=datapack[3]*256+datapack[4];
      rotation=datapack[5]*256+datapack[6];
      
      if(velocity>255) velocity=255;
      if(velocity<-255) velocity=-255;
      
      while(angle>=360) 
      {
        angle-=360;
      }
      while(angle<0) 
      {
        angle+=360;
      }
      
      if(rotation>255) rotation=255;
      if(rotation<-255) rotation=-255;
      
      Trigonometry();
    }
    command=255;  
    return;
  }

  if(command==4 && packsize==6) //============================================= Serial port configuration ===========================================================
  {
    baudrate[0]=datapack[1]*256U+datapack[2];
    baudrate[1]=datapack[3]*256U+datapack[4];
    sermode=datapack[5];
    Serial.begin(baudrate[mcu]);                                             // change serial port baud rate
    EEPROMsave();                                                            // update EEPROM
    command=255;
    return;
  }

  if(command==5 && packsize>2 && packsize<33) //=============================== Send Serial Data =====================================================================
  {
    if((mcu+1)==datapack[1])
    {
      for(byte i=0;i<(packsize-2);i++)
      {
        serpack[i]=datapack[i+2];
      }
      Serial.write(serpack,packsize-2);
    }
    command=255;
    return;
  }  

  if(command==6 && packsize==2) //============================================= Status request ======================================================================
  {
                                                                             // each mcu sends it's data seperately to minimize interferance with motor speed control
                                                                             
    byte spsize=0;                                                           // intitial send pack size = 0
    int request=datapack[1];                                                 // copy datapack to global variable "request" ASAP so datapack can be reused
    
    if(request&1)// Bit 0:                                                   // return encoder counter values
    {
      sendpack[0] =highByte(acount);
      sendpack[1] = lowByte(acount);
      sendpack[2] =highByte(bcount);
      sendpack[3] = lowByte(bcount);
      spsize=4;                                                              // increment pack size by 4 bytes (counts from 2 encoders only)
    }

    if(request&2)// Bit 1:                                                   // reset encoder counters
    {
      acount=0;
      bcount=0;
    }

    if(request&4)// Bit 2:                                                   // return motor currents
    {
      sendpack[spsize+0] =highByte(analogvalue[0]);
      sendpack[spsize+1] = lowByte(analogvalue[0]);
      sendpack[spsize+2] =highByte(analogvalue[1]);
      sendpack[spsize+3] = lowByte(analogvalue[1]);
      spsize+=4;                                                             // increment pack size by 4 bytes (current from 2 motors only)
    }

    if(((request&8) && mcu==0) || ((request&16) && mcu==1))// Bits 3&4:      // return MCU analog values
    {
      sendpack[spsize+0]=highByte(analogvalue[2]);
      sendpack[spsize+1]= lowByte(analogvalue[2]);
      sendpack[spsize+2]=highByte(analogvalue[3]);
      sendpack[spsize+3]= lowByte(analogvalue[3]);
      sendpack[spsize+4]=highByte(analogvalue[4]);
      sendpack[spsize+5]= lowByte(analogvalue[4]);
      spsize+=6;
    }

    if(request&32)// Bit 5:                                                  // return error log
    {
      sendpack[spsize]=eflag;
    }

    if(request&64)// Bit 6:                                                  // clear error log
    {  
      eflag=0;
    }
    
    byte returnaddress=master;                                               // return address is I²C master by default
    if((request&127) && mcu==0) returnaddress=address+1;                     // bit 7 indicates internal request - return to other processor
    if((request&127) && mcu==1) returnaddress=address-1;                     // bit 7 indicates internal request - return to other processor
    
    Wire.beginTransmission(returnaddress);
    Wire.write(sendpack,spsize);
    Wire.endTransmission();
    command=255;
  }
}


