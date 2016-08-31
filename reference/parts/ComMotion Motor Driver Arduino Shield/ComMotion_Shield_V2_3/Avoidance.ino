void Avoidance()
{
  static int sensor;
  
  velocity=50;
  rotation=120;
  if(mcu==1)                                         // demo controlled via MCU2 which has sensor connected to A7
  {
    if(analog==0)
    {
      sensor=analogRead(senspin);                    // read current sensor value
    }
    
    if(sensor<900) time=millis();
        
    if(analog==3)
    {
      angle=int((millis()-time)*360L/2300L)+155;
      Wire.beginTransmission(address-1);
      Wire.write(7);
      Wire.write(highByte(angle));
      Wire.write(lowByte(angle));
      Wire.endTransmission();
    }
  }
  Trigonometry();
}
