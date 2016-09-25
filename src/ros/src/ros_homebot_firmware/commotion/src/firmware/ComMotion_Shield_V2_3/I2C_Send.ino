
// This is called from the I2C host.
// Wire supports sending at most 32 bytes at a time.
void I2C_Send()
{
  
  byte myArray[REQUESTFOR_SIZE];
  
  myArray[0] = lowByte(acount);
  myArray[1] = highByte(acount);
  myArray[2] = lowByte(bcount);
  myArray[3] = highByte(bcount);
  myArray[4] = eflag;
  myArray[5] = lowByte(mspeed[motora]);
  myArray[6] = highByte(mspeed[motora]);
  myArray[7] = lowByte(mspeed[motorb]);
  myArray[8] = highByte(mspeed[motorb]);
  
  /*
  myArray[0] = mcu;
  myArray[1] = 0;
  myArray[2] = mcu;
  myArray[3] = 0;
  */
  
//  acount_abs += 1;
//  bcount_abs += 1;
  /*
  myArray[0] = lowByte(acount_abs);
  myArray[1] = highByte(acount_abs);
  myArray[2] = lowByte(bcount_abs);
  myArray[3] = highByte(bcount_abs);
  */
  
  Wire.write(myArray, REQUESTFOR_SIZE);
  
}

