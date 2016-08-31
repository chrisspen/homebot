/*
If the host's connection doesn't reset the Arduino, then the number the Arduino
prints should not change when the hosts disconnects and reconnects.

Meanwhile, LED 13 will blink once per second.
*/

unsigned long startup;
unsigned long count;

void setup()
{
    Serial.begin(57600); // must match ino.ini
    pinMode(13, OUTPUT);
    randomSeed(analogRead(0));
    startup = random(10000);
    count = 0;
}

void loop()
{
    digitalWrite(13, !digitalRead(13));
    Serial.println(String("startup:")+String(startup)+String(' ')+String(count));
    Serial.flush();
    delay(1000);
    count += 1;
}
