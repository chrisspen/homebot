// https://www.arduino.cc/en/Tutorial/MasterReader
// http://arduino.stackexchange.com/a/3388/4478

void send_wire_int(int num){

  // Uncomment below if you want so send back in same format
  // Wire.write(0);

  // Send low byte
  Wire.write((uint8_t)num);

  // Send high byte
  num >>= 8;
  Wire.write((uint8_t)num);
}

void I2C_Send()
{
	// Send encoder A count.
	send_wire_int(acount);
  
	// Send encoder B count.
	send_wire_int(bcount);
}
