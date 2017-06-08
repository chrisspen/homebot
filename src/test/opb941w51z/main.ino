
#define IR_SENSOR_PIN A0
#define LED_PIN 13

void setup() {
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    // set pullup on analog pin 0
    // Because the signal pin is wired to ground, we need to enable the pullup resistor to form a voltage divider.
    pinMode(IR_SENSOR_PIN, INPUT_PULLUP);

    // start serial connection
    Serial.begin(57600); // must match ino.ini
}

void loop() {

    digitalWrite(LED_PIN, digitalRead(IR_SENSOR_PIN));
    Serial.println(digitalRead(IR_SENSOR_PIN));
    delay(100);
  
}
