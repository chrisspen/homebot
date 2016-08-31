#line 2 "test.ino"
#include <ArduinoUnit.h>

#include "Smooth.h"
/*
test(ok) 
{
  int x=3;
  int y=3;
  assertEqual(x,y);
}

test(bad)
{
  int x=3;
  int y=3;
  assertNotEqual(x,y);
}*/

test(servo_smooth){
	verbosity = TEST_VERBOSITY_ALL;
	
	long start_pos, end_pos, dist;
	int speed = 1; //# sec/pos
	int v, t, resolution;
	
	start_pos = 90;
	end_pos = 60;
	speed = 30;
	resolution = 10;
	Test::out->println(String("start=")+String(start_pos)+String(" end=")+String(end_pos)+String(" speed=")+String(speed));
	for(t=0; t < (abs((end_pos - start_pos)/speed)+1)*1000; t=t+1000/resolution){
	    v = get_servo_signal(
	    	start_pos,//start
			end_pos,//end
			speed,//speed
			t/1000.//t
		);
		Test::out->println(String("t=")+String(t)+String(" v=")+String(v));
	}
	assertEqual(t, 2000);
	assertEqual(v, 60);
	/*
	start_pos = 60;
	end_pos = 90;
	speed = 1;
	Test::out->println(String("start=")+String(start_pos)+String(" end=")+String(end_pos)+String(" speed=")+String(speed));
	for(t=0; t<(int)abs((end_pos - start_pos)/speed)+1; t++){
	    v = get_servo_signal(
	    	start_pos,//start
			end_pos,//end
			speed,//speed
			t//t
		);
		Test::out->println(String("t=")+String(t)+String(" v=")+String(v));
	}
	assertEqual(t, 31);
	assertEqual(v, 90);
	
	start_pos = 60;
	end_pos = 90;
	speed = 3;
	Test::out->println(String("start=")+String(start_pos)+String(" end=")+String(end_pos)+String(" speed=")+String(speed));
	for(t=0; t<(int)abs((end_pos - start_pos)/speed)+1; t++){
	    v = get_servo_signal(
	    	start_pos,//start
			end_pos,//end
			speed,//speed
			t//t
		);
		Test::out->println(String("t=")+String(t)+String(" v=")+String(v));
	}
	assertEqual(t, 11);
	assertEqual(v, 90);
*/
}

void setup()
{
  Serial.begin(9600);
  while(!Serial); // for the Arduino Leonardo/Micro only
}

void loop()
{
  Test::run();
}
