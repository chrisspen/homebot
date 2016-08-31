
#define IR_SENSOR_PIN A0
#define LED_PIN 13

bool led_state = false;
int sensorValue = 0;  // variable to store the value coming from the sensor

//int threshold = 512;//a
int threshold = 600;//b

void setup() {

    // start serial connection
    Serial.begin(57600); // must match ino.ini
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, led_state);
    
    // set pullup on analog pin 0
    // Because the signal pin is wired to ground, we need to enable the pullup resistor to form a voltage divider.
    pinMode(IR_SENSOR_PIN, INPUT);
    digitalWrite(IR_SENSOR_PIN, HIGH);

}

void loop() {

    // read the value from the sensor:
    sensorValue = analogRead(IR_SENSOR_PIN);
    
    led_state = (sensorValue < threshold)? true : false;
    digitalWrite(LED_PIN, led_state);
    
    //Serial.println(sensorValue);
    //delay(1000);
  
}
