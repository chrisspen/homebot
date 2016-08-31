
//============================================================== Receive I²C command from external device or other MCU =======================

void I2C_Receive(int bytes)                                   // received command as I²C slave     
{
  for(byte i=0;i<bytes;i++)                                 
  {
    datapack[i]=Wire.read();                                  // transfer data from I²C buffer to datapack
  }
  
  command=datapack[0];
  packsize=bytes;
  
  //Serial.print("Pack size:");Serial.println(bytes,DEC);
  //Serial.print("Command:");Serial.println(command,DEC);
  
  if(mode==1 && command<5)
  {
    mode=0;                                                   // disable demo mode
    EEPROM.write(1,mode);
  }
  
    
  //------------------------------------------------------------ demo syncronization ---------------------------------------------------------
  
  if(command==15 && packsize==3)
  {
    EEPROM.write(0,datapack[1]);                              // only update EEPROM if necessary
    EEPROM.write(1,datapack[2]);                              // only update EEPROM if necessary
    
    demo=datapack[1];
    mode=datapack[2];
    
    command=255;
    return;
  }
  
  if(command==7 && packsize==3)  //============================= Demo Modes Angle Update ======================================================
  {
    angle=datapack[1]*256+datapack[2];
    Trigonometry();
    command=255;
    return;
  }
}
  
  


