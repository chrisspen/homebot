
//============================================================================== Generate 2kHz beeps =======================================================

void Beep(byte beeps)
{
  
  
  for(int b=0;b<beeps;b++)                                                    // loop to generate multiple beeps
  {
    PORTB=PORTB|B00001111;                                                    // set all motor direction and PWM pins high
    for(int duration=0;duration<600;duration++)                               // generate 2kHz tone for 200mS
    {
      delayMicroseconds(250);                                                 // wait 250uS to generate 2kHz tone
      PORTB=PORTB^B00001001;                                                  // toggle direction bits
    }
    PORTB=PORTB&B11110000;                                                    // turn off all motors
    delay(200);                                                               // pause for 200mS (1/5th of a second) between beeps
  }
  
}

//============================================================================== Plays Tune ===============================================================

void Tune()                                                                   // modified code from Brett Hagman's original Tone library examples
{
  int tempo=700;                                                              // changes the tempo (speed) that the music plays at
  int notes=29;                                                               // number of notes to be played

  int melody[]={                                                              // define melody (Row, row, row your boat)
  NOTE_C6,NOTE_C6,NOTE_C6,NOTE_D6,NOTE_E6,0,
  NOTE_E6,NOTE_D6,NOTE_E6,NOTE_F6,NOTE_G6,0,
  NOTE_C7,NOTE_C7,NOTE_C7,NOTE_G6,NOTE_G6,NOTE_G6,NOTE_E6,NOTE_E6,NOTE_E6,NOTE_C6,NOTE_C6,NOTE_C6,
  NOTE_G6,NOTE_F6,NOTE_E6,NOTE_D6,NOTE_C6
  };

  byte noteDurations[]={                                                      // define the duration of each note here
  2,2,2,4,2,4,
  2,4,2,4,2,4,
  4,4,4,4,4,4,4,4,4,4,4,4,
  2,4,2,4,2
  };
  
  for (byte Note = 0; Note < notes; Note++)                                   // Play melody
  {
    long pulselength = 1000000L/melody[Note];
    long noteDuration = tempo/noteDurations[Note];
    long pulses=noteDuration*tempo/pulselength;
    
    if (pulselength>100000L)                                                  // a note of 0 generates a pause
    {
      delay(noteDuration);
    }
    else
    {
      for(int p=0;p<pulses;p++)
      {                                                                      
        PORTB=PORTB|B00001111;                                                // set all motor direction and PWM pins high
        delayMicroseconds(pulselength/2-20);                                  // frequency of note divide by 2
        
                                                                              // drive motors backward
        PORTB=PORTB&B11110110;                                                // drive motors backward - set direction bits low
        delayMicroseconds(pulselength/2-20);                                  // frequency of note divide by 2
      }
      
      PORTB=PORTB&B111000;                                                    // turn off motors
      delay(noteDuration * 3 / 10);                                           // short pause between notes
    }
  }
}
