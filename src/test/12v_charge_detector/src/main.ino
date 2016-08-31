
#define LED_PIN 13
#define EXTERNAL_POWER_DETECT_PIN A2

bool external_power_detected = false;

void setup() {

    // start serial connection
    Serial.begin(57600); // must match ino.ini
    
    pinMode(LED_PIN, OUTPUT);
    pinMode(EXTERNAL_POWER_DETECT_PIN, INPUT);

}

void loop(){

    external_power_detected = digitalRead(EXTERNAL_POWER_DETECT_PIN);

    if(external_power_detected){
        digitalWrite(LED_PIN, HIGH);
    }else{
        digitalWrite(LED_PIN, LOW);
    }
    
    Serial.println(String("EXTERNAL POWER: ") + String(external_power_detected));
    
    delay(100);
      
}
