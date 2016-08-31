
#include <math.h>

#define THERMISTOR_ADC_PIN A0

double Thermistor(int RawADC) {
    double Temp;
    Temp = log(10000.0*((1024.0/RawADC-1))); 
//         =log(10000.0/(1024.0/RawADC-1)) // for pull-up configuration
    Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp ))* Temp );
    Temp = Temp - 273.15;            // Convert Kelvin to Celcius
    Temp = (Temp * 9.0)/ 5.0 + 32.0; // Convert Celcius to Fahrenheit
    return Temp;
}

void setup() {
    Serial.begin(57600);
    pinMode(THERMISTOR_ADC_PIN, INPUT);
}

void loop() {
    Serial.println(int(Thermistor(analogRead(THERMISTOR_ADC_PIN))));  // display Fahrenheit
    delay(1000);
}
