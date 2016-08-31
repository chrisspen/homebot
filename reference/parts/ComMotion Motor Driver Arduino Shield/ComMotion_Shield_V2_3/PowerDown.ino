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
