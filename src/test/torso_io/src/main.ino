
#include <Wire.h>

#include "Thread.h"
#include "ThreadController.h"

#include "arduino_pinout.h"

#include "BatteryVoltageSensor.h"
#include "BatteryTemperatureSensor.h"
#include "BumperSensor.h"
#include "EdgeSensor.h"
#include "ExternalPowerSensor.h"
#include "MotionController.h"
#include "PowerController.h"
#include "StatusButtonSensor.h"
#include "StatusLightController.h"
#include "UltrasonicSensor.h"

// Sensor declarations

BumperSensor bumper_left_sensor = BumperSensor(BUMPER_L_PIN);
BumperSensor bumper_middle_sensor = BumperSensor(BUMPER_M_PIN);
BumperSensor bumper_right_sensor = BumperSensor(BUMPER_R_PIN);
EdgeSensor edge_left_sensor = EdgeSensor(EDGE_L_PIN);
EdgeSensor edge_middle_sensor = EdgeSensor(EDGE_M_PIN);
EdgeSensor edge_right_sensor = EdgeSensor(EDGE_R_PIN);
UltrasonicSensor sonic_left_sensor = UltrasonicSensor(SONIC_L_PIN);
UltrasonicSensor sonic_middle_sensor = UltrasonicSensor(SONIC_M_PIN);
UltrasonicSensor sonic_right_sensor = UltrasonicSensor(SONIC_R_PIN);
StatusButtonSensor status_button = StatusButtonSensor(SIGNAL_BUTTON_PIN);
BatteryVoltageSensor battery_voltage_sensor = BatteryVoltageSensor(
    // The battery consists of 3 * 4.2V lipo cells.
    12.6,
    2000000, // ohms
    1000000, // ohms
    BATTERY_VOLTAGE_PIN,
    // The battery is considered dead when 80% discharged.
    0.8
);
BatteryTemperatureSensor battery_temperature_sensor = BatteryTemperatureSensor(BATTERY_TEMP_PIN);
ExternalPowerSensor external_power_sensor = ExternalPowerSensor(EXTERNAL_POWER_SENSE_PIN);

// Controller declarations.

StatusLightController status_light = StatusLightController(STATUS_LED_PIN);
MotionController motion_controller;
PowerController power_controller = PowerController(POWER_OFF_PIN, status_button, status_light);

class SonarThread: public Thread
{
    public:

        void run(){
            
            // Space each reading to prevent echo from interfering
            // with sequential readings.
            int spacing = 20;
        
            sonic_left_sensor.update();
            delay(spacing);
            
            sonic_middle_sensor.update();
            delay(spacing);
            
            sonic_right_sensor.update();
            delay(spacing);
            
            runned();
        }
};

class BumperThread: public Thread
{
    public:

        void run(){
            
            bumper_left_sensor.update();
            bumper_middle_sensor.update();
            bumper_right_sensor.update();
            
            runned();
        }
};

class EdgeThread: public Thread
{
    public:

        void run(){
            
            edge_left_sensor.update();
            edge_middle_sensor.update();
            edge_right_sensor.update();
            
            runned();
        }
};

class BatteryThread: public Thread
{
    public:

        void run(){
            
            battery_voltage_sensor.update();
            external_power_sensor.update();
            battery_temperature_sensor.update();
            
            runned();
        }
};

class PowerThread: public Thread
{
    public:

        void run(){
            
            power_controller.update();
            
            runned();
        }
};

class OutputThread: public Thread
{
    public:
    
        bool blink = false;

        void run(){
        
            status_button.update();
            
            Serial.println("");

            Serial.println(String("sonar.left:")+String(sonic_left_sensor.microsecondsToCentimeters()));
            Serial.println(String("sonar.middle:")+String(sonic_middle_sensor.microsecondsToCentimeters()));
            Serial.println(String("sonar.right:")+String(sonic_right_sensor.microsecondsToCentimeters()));
            
            Serial.println(String("bumper.left:")+String(bumper_left_sensor.has_contact()));
            Serial.println(String("bumper.middle:")+String(bumper_middle_sensor.has_contact()));
            Serial.println(String("bumper.right:")+String(bumper_right_sensor.has_contact()));
            
            Serial.println(String("edge.left:")+String(edge_left_sensor.has_contact()));
            Serial.println(String("edge.middle:")+String(edge_middle_sensor.has_contact()));
            Serial.println(String("edge.right:")+String(edge_right_sensor.has_contact()));
            
            Serial.println(String("status.button:")+String(status_button.is_pressed()));
            Serial.println(String("battery.voltage:")+String(battery_voltage_sensor.get_voltage()));
            Serial.println(String("battery.charge:")+String(battery_voltage_sensor.get_charge_ratio()));
            Serial.println(String("battery.temperature:")+String(battery_temperature_sensor.get_temperature_fahrenheit()));
            Serial.println(String("battery.connected:")+String(external_power_sensor.is_connected()));
            
            Serial.println("");
            Serial.flush();
        
            //Serial.println(String("test:")+String(millis()));
            //delay(500);
            
            //digitalWrite(13, blink);
            //blink = !blink;
            
            runned();
        }
};

SonarThread sonar_thread = SonarThread();
BumperThread bumper_thread = BumperThread();
EdgeThread edge_thread = EdgeThread();
BatteryThread battery_thread = BatteryThread();
PowerThread power_thread = PowerThread();
OutputThread output_thread = OutputThread();
ThreadController thread_controller = ThreadController();

void setup(){
    
    Serial.begin(57600); // must match ino.ini
    
    Wire.begin(1);      // Join I²C bus as Master with address #1

    // We initialize here so we can be sure it's done after the I2C initialization.
    motion_controller = MotionController();

    // Sensor threads.
    sonar_thread.setInterval(50);
    thread_controller.add(&sonar_thread);
    
    bumper_thread.setInterval(1);
    thread_controller.add(&bumper_thread);
    
    edge_thread.setInterval(1);
    thread_controller.add(&edge_thread);
    
    battery_thread.setInterval(100);
    thread_controller.add(&battery_thread);
    
    // Output threads.
    power_thread.setInterval(10);
    thread_controller.add(&power_thread);
    output_thread.setInterval(250);
    thread_controller.add(&output_thread);
    
}

void loop(){

    thread_controller.run();

}
