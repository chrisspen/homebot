void LineFollow()
{
  static byte edgedetect;
  static int sensor;
  
  velocity=40*(angle>0);
  rotation=120;
  if(mcu==1)                                         // demo controlled via MCU2 which has sensor connected to A7
  {
    if(analog==0)
    {
      sensor=analogRead(senspin);                    // read current sensor value
    }
    
    if(analog==1)
    {
      if((sensor>900 && (edgedetect&1)==0) || (sensor<800 && (edgedetect&1)==1)) edgedetect++;
      if(edgedetect>3) 
      {
        time=millis();
        edgedetect=0;
        angle+=60;
        if(angle>-300) angle=0;
      }
    }
    
    if(angle>-300 && analog==3)
    {
      
      angle=int((millis()-time)*360L/2300L);
      Wire.beginTransmission(address-1);
      Wire.write(7);
      Wire.write(highByte(angle));
      Wire.write(lowByte(angle));
      Wire.endTransmission();
    }
  }
  Trigonometry();
}
    




