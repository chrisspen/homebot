
#define RGB_PIN_RED     6
#define RGB_PIN_GREEN   7
#define RGB_PIN_BLUE    8

void setup() {

    Serial.begin(57600);
    
    pinMode(RGB_PIN_RED, OUTPUT);
    pinMode(RGB_PIN_GREEN, OUTPUT);
    pinMode(RGB_PIN_BLUE, OUTPUT);
    
}

void set_rgb(bool r, bool g, bool b){
    digitalWrite(RGB_PIN_RED, r);
    digitalWrite(RGB_PIN_GREEN, g);
    digitalWrite(RGB_PIN_BLUE, b);
}

void loop() {


    Serial.println(String("red"));Serial.flush();
    set_rgb(1, 0, 0);
    delay(1000);

    Serial.println(String("green"));Serial.flush();
    set_rgb(0, 1, 0);
    delay(1000);
    
    Serial.println(String("blue"));Serial.flush();
    set_rgb(0, 0, 1);
    delay(1000);

    Serial.println(String("white"));Serial.flush();
    set_rgb(1, 1, 1);
    delay(1000);

}
