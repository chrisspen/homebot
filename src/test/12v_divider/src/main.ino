
#define BATTERY_SENSOR_DETECT_PIN = A0;
#define BATTERY_SENSOR_ENABLE_PIN = 12;

int raw_battery_voltage = 0;
float pin_voltage = 0; // the calculated voltage on the ADC pin
float battery_voltage = 0; // implied voltage of the battery
float battery_voltage_ratio = 2.4; // Change this to match the MEASURED ration of the circuit

void setup() {

    // start serial connection
    Serial.begin(57600); // must match ino.ini
    
    pinMode(BATTERY_SENSOR_DETECT_PIN, INPUT);
    pinMode(BATTERY_SENSOR_ENABLE_PIN, OUTPUT);

}

void loop() {

    // Enable battery sensor.
    digitalWrite(BATTERY_SENSOR_ENABLE_PIN, HIGH);
    
    // read the value from the sensor:
    raw_battery_voltage = analogRead(BATTERY_SENSOR_DETECT_PIN);
    
    // Disable battery sensor.
    digitalWrite(BATTERY_SENSOR_ENABLE_PIN, LOW);
    
    // Calculate the voltage on the A/D pin
    // A reading of 1 for the A/D = 0.0048mV
    // if we multiply the A/D reading by 0.00488 then 
    // we get the voltage on the pin.
    // The number 0.00488 is the volts per division for the analog conversion.
    // 5V / 1024 divisions = 0.00488 volts per division 
    pin_voltage = raw_battery_voltage * 0.00488;
    
    battery_voltage = pin_voltage * battery_voltage_ratio;
    Serial.println(battery_voltage);
    
    delay(1000);
  
}
