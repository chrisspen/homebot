//
// ServoEaser0ManualEasing.ino - show how to send manual easing commands
//
// 2011, TeamPneumo, Tod E. Kurt, http://todbot.com/blog/
//
//

#include <Servo.h>
#include "ServoEaser.h"

#define SWEEP_MS 2000

const int ledPin   = 13; 
const int servoPin = 6;

int servoFrameMillis = 20;  // minimum time between servo updates

Servo servo1; 
ServoEaser servoEaser;

bool state = true;

//
void setup()
{
  Serial.begin(57600);
  Serial.println("ServoEaser0ManualEasing");

  // first, get the servo ready
  servo1.attach( servoPin );

  //Serial.println("moving to 90 degrees immediately");
  // begin with a framerate, no starting position, we don't know
  servoEaser.begin( servo1, servoFrameMillis );
  //servoEaser.setMinMaxMicroseconds(1400-600, 1400+600);
  servoEaser.useMicroseconds( true );  // fine-control mode

  // do manual easing
  Serial.println("moving to 30 degrees over 5 seconds");
  servoEaser.easeTo( 90-60, SWEEP_MS);
}

//
void loop()
{
  servoEaser.update();

  printCurrPos();

  if( servoEaser.hasArrived() ){
	  if(state){
		  servoEaser.easeTo(90+60, SWEEP_MS);
		  //servoEaser.easeTo(90, SWEEP_MS);
	  }else{
		  servoEaser.easeTo(90-60, SWEEP_MS);
	  }
	  state = !state;
  }
    
}

void printCurrPos()
{
    static long nextPrintTime;
    
    if( (long)(millis() - nextPrintTime) >= 0 ) {
        nextPrintTime += 200; // 100 millisecs between print statements
        Serial.print("currPos: ");
        Serial.println( servoEaser.getCurrPos() );
    }
}
