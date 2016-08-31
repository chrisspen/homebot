
#define STATUS_LED_PIN              13
#define HEAD_POWER                  A3

void setup()
{
    
    Serial.begin(57600);
    pinMode(STATUS_LED_PIN, OUTPUT);
    pinMode(HEAD_POWER, OUTPUT);

}

void loop()
{

    digitalWrite(STATUS_LED_PIN, HIGH);
    digitalWrite(HEAD_POWER, HIGH);
    delay(1000);
    
    digitalWrite(STATUS_LED_PIN, LOW);
    digitalWrite(HEAD_POWER, LOW);
    delay(1000);
    
}
