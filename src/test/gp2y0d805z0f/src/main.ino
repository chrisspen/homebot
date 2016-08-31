
#define SENSOR_PIN 12

int val = 0;

void setup(){
    
    Serial.begin(57600); // must match ino.ini
    
    pinMode(SENSOR_PIN, INPUT);
}

void loop(){
    val = digitalRead(SENSOR_PIN);
    Serial.println(((String)"Sensor: ") + val);
    delay(1000);
}
