
void Commands()
{
  if(mcu==0 && (command<5 || command==CMD_RESET_EEPROM)) // if this is MCU1 and command is 0-4 then repeat command to MCU2 
  {
    Wire.beginTransmission(address+1);                                       // take control of I²C bus and address MCU2
    Wire.write(datapack,packsize);                                           // relay commands to MCU2
    Wire.endTransmission();                                   
  }
  
  if(command==CMD_RESET_EEPROM)
  {
    EEPROMdefaults();
    command = CMD_NULL;
    return;
  }
  
  if(command>15) command-=16;                                                // 16 is added (bit 3 set high) for internal commands (no repeat)
  
  //============================================ Basic Configuration Data Received =================================================
  if(command==CMD_SET_BASIC_CONFIG && packsize==10)
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
    command = CMD_NULL;
    return;
  }

  //============================================================ Encoder Configuration Data Received =================================================
  if(command==CMD_SET_ENCODER_CONFIG) 
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
    command = CMD_NULL;
    return;
  }

  //============================ Motor Control =======================================================================
  if(command==CMD_SET_MOTOR_CONFIG && packsize==9) 
  {
    //if(configuration==CHASSIS_CONFIG_INDIVIDUAL  packsize==9)               // Individual motor control  
    //{
      for(byte i=0; i<4; i++)
      {
        mspeed[i] = (int)(datapack[i*2+1]*256 + datapack[i*2+2]);
        //TODO:remove? unreliable?
        if(mspeed[i] > 0){
          mdir[i] = 1;
        }else if(mspeed[i] < 0){
          mdir[i] = -1;
        }else{
          mdir[i] = 0;
        }
      }
    //}
    command = CMD_NULL;  
    return;
  }
  
}
