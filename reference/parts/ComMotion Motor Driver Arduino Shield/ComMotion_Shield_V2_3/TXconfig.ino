void TXconfig()
{
  if(mcu)
  {
    //Serial.begin(57600);
    Serial.println("");
    Serial.print("Demo:");Serial.println(demo,DEC);
    Serial.print("Mode:");Serial.println(mode,DEC);
    Serial.print("Configuration:");Serial.println(configuration,DEC);
    Serial.println("");
    Serial.print("Lo Bat:");Serial.print(lowbat/10.0);Serial.println("V");
    Serial.print("M1 maximum current:");Serial.print(maxamps[0]/100.0);Serial.println("A");
    Serial.print("M2 maximum current:");Serial.print(maxamps[1]/100.0);Serial.println("A");
    Serial.print("M3 maximum current:");Serial.print(maxamps[2]/100.0);Serial.println("A");
    Serial.print("M4 maximum current:");Serial.print(maxamps[3]/100.0);Serial.println("A");
    Serial.println("");
    Serial.print("Master I2C address:");Serial.println(master,DEC);
    Serial.print("I2C address offset:");Serial.println(addroffset,DEC);
    Serial.print("I2C clock frequency:");Serial.print(i2cfreq*300+100);Serial.println("Kb/sec");
    Serial.println("");
    Serial.print("M1 maximum speed:");Serial.print(motrpm[0]);Serial.println("RPM");
    Serial.print("M2 maximum speed:");Serial.print(motrpm[1]); Serial.println("RPM");
    Serial.print("M3 maximum speed:");Serial.print(motrpm[2]);Serial.println("RPM");
    Serial.print("M4 maximum speed:");Serial.print(motrpm[3]);Serial.println("RPM");
    Serial.println("");
    Serial.print("M1 encoder resolution:");Serial.println(encres[0]/100.0);
    Serial.print("M2 encoder resolution:");Serial.println(encres[1]/100.0);
    Serial.print("M3 encoder resolution:");Serial.println(encres[2]/100.0);
    Serial.print("M4 encoder resolution:");Serial.println(encres[3]/100.0);
    Serial.println("");
    Serial.print("M1 maximum encoder pulse:");Serial.print(maxpulse[0]/255L);Serial.println("uS");
    Serial.print("M2 maximum encoder pulse:");Serial.print(maxpulse[1]/255L);Serial.println("uS");
    Serial.print("M3 maximum encoder pulse:");Serial.print(maxpulse[2]/255L);Serial.println("uS");
    Serial.print("M4 maximum encoder pulse:");Serial.print(maxpulse[3]/255L);Serial.println("uS");
    Serial.println("");
    Serial.print("M1 reserve power:");Serial.print(reserve[0],DEC);Serial.println("%");
    Serial.print("M2 reserve power:");Serial.print(reserve[1],DEC);Serial.println("%");
    Serial.print("M3 reserve power:");Serial.print(reserve[2],DEC);Serial.println("%");
    Serial.print("M4 reserve power:");Serial.print(reserve[3],DEC);Serial.println("%");
    Serial.println("");
    Serial.print("M1 stall time:");Serial.print(stalltm[0]);Serial.println("mS");
    Serial.print("M2 stall time:");Serial.print(stalltm[1]);Serial.println("mS");
    Serial.print("M3 stall time:");Serial.print(stalltm[2]);Serial.println("mS");
    Serial.print("M4 stall time:");Serial.print(stalltm[3]);Serial.println("mS");
    Serial.println("");
    Serial.print("Serial mode:");Serial.println(sermode,DEC);
    Serial.print("Baud rate port 1:");Serial.println(baudrate[0]);
    Serial.print("Baud rate port 2:");Serial.println(baudrate[1]);
    Serial.print("Defaults:");Serial.println(EEPROM.read(40),DEC);
    Serial.println("");
    Serial.println("");
  }
}

