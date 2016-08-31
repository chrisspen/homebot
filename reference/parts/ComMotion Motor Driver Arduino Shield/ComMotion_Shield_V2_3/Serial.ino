//======================================================================================== Takes data from serial port and redirects it as determined by serial mode =======================================
void SerialInput()                                                        
{
  if((mcu==0 && (sermode==1 || sermode==3)) || (mcu==1 && (sermode==2 || sermode==4)))  // Command received on serial port
  {
    byte i=0;
    command=255;
    while(Serial.available()>0)
    {
      datapack[i]=Serial.read();                                                        // data is read from the serial buffer and added to the command datapack
      i++;
    }
    
    //------------------------------------------------------------------------------------ Verify that command data has been received ---------------------------------------------------------------------
    if(datapack[0]==0 && i==10)
    {
      command=datapack[0];
      packsize=i;
    }
    
    if(datapack[0]==1 && (i==7 || i==25))
    {
      command=datapack[0];
      packsize=i;
    }
    
    if(datapack[0]==2 && (i==6 || i==8))
    {
      command=datapack[0];
      packsize=i;
    }
    
    if(datapack[0]==3 && i==6)
    {
      command=datapack[0];
      packsize=i;
    }
    
    if(datapack[0]==4 && i>2)
    {
      command=datapack[0];
      packsize=i;
    }
    
    if(command==255)
    {
      Serial.println("Command not recognized!");
      Serial.print("Command:");Serial.print(datapack[0],DEC);
      Serial.print("Pack Size:");Serial.println(i,DEC);
      Serial.println("");
    }
    else
    {
      if(mode==1)
      {
        mode=0;                                                                         // disable demo mode
        EEPROM.write(1,mode);
      }
    }
  }
  else
  {
    byte j=4;                                                                           // byte 1-4 is serial pack header
    serpack[0]=83;                                                                      // this header is used to indicate
    serpack[1]=80;                                                                      // the start of a new data packet
    serpack[2]=mcu+49;                                                                  // and where the data came from
    serpack[3]=58;                                                                      // "SP1:" or "SP2:" is serial port 1 or 2
    while(Serial.available()>0)
    {
      serpack[j]=Serial.read();                                                         // data is read from the serial buffer and added to the datapack
      j++;
    }
    
    if(sermode==0 || (sermode==1 && mcu==1) || (sermode==2 && mcu==0))                  // send serial data to I²C master
    {
      Wire.beginTransmission(master);                                                   // address I²C master
      Wire.write(serpack,j);                                                            // send data with 4 byte header
      Wire.endTransmission();                                                           // release I²C bus
    }
    else if((configuration==3 && mcu==1) || (configuration==4 && mcu==0))               // pass serial data to other serial port
    {
      byte pass=master;
      if(mcu==0) pass=address+1;                                                        // address of MCU to pass data to
      if(mcu==1) pass=address-1;
      Wire.beginTransmission(pass);                                                     
      Wire.write(20);                                                                   // command to internal send serial data (4+16)
      Wire.write(j);                                                                    // serial pack size including header data
      Wire.write(serpack,j);                                                            // send serialpack
      Wire.endTransmission();
    }
  }
  
  
  
  
  
}
