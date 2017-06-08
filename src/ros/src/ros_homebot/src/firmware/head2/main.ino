/*
 * Copyright 2017 Chris Spencer
 */
// http://wiki.ros.org/rosserial_arduino/Tutorials/Hello%20World
//#include "Arduino.h"
#include <ros.h>
#include <math.h>
#include <stdint.h>
//#include <tf/tf.h>
//#include <tf/transform_broadcaster.h>
//#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>
//#include <std_msgs/Float32MultiArray.h>
//#include <std_msgs/Int32MultiArray.h>
//#include <std_msgs/Int16MultiArray.h>
//#include <std_msgs/Int8MultiArray.h>
//#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/String.h>
//#include <geometry_msgs/Twist.h>
//#include <diagnostic_msgs/DiagnosticArray.h>
//#include <diagnostic_msgs/DiagnosticStatus.h>
//#include <diagnostic_msgs/KeyValue.h>
//#include <Wire.h>

#include "ArduinoPinout.h"
#include "I2CAddresses.h"
#include "StringUtils.h"
#include "PanController.h"
#include "TiltController.h"

//#define DIAGNOSTIC_STATUS_LENGTH 1
//#define DIAGNOSTIC_REPORT_FREQ_MS 1000

PanController pan_controller = PanController(PAN_MOTOR_ENABLE, PAN_MOTOR_PHASE, PAN_MOTOR_POSITION_REF);
TiltController tilt_controller = TiltController(TILT_SERVO_POS_SET, TILT_SERVO_POS_GET);

// Convert the BNO055's calibration code [0-3] to a diagnostics status.
//int calib_to_status[] = {
    //diagnostic_msgs::DiagnosticStatus::ERROR, //2, raw 0=uncalibrated
    //diagnostic_msgs::DiagnosticStatus::WARN,  //1, raw 1=partial
    //diagnostic_msgs::DiagnosticStatus::WARN,  //1, raw 2=almost
    //diagnostic_msgs::DiagnosticStatus::OK     //0, raw 3=completely calibrated
//};

// If set to true, all motors will be stopped immediately.
//bool halt = false;

// If true, all sensors will push their current readings to the host, even if they haven't changed
// since last polling.
//bool force_sensors = false;

// Records when we last received a message from rosserial on the host.
// This is used to detect a lost connection to the host and perform emergency shutdown of the motors.
bool connected = false;

// http://wiki.ros.org/roscpp/Overview/Logging
// http://wiki.ros.org/rosserial/Overview/Logging
ros::NodeHandle nh;

//http://wiki.ros.org/std_msgs
//http://wiki.ros.org/common_msgs
//std_msgs::Int16 int16_msg;
//std_msgs::Byte byte_msg;
//std_msgs::Bool bool_msg;
//std_msgs::Float32 float_msg;
//std_msgs::String string_msg;
//std_msgs::UInt16MultiArray uint16ma_msg;
//sensor_msgs::Range range_msg;
//sensor_msgs::BatteryState battery_msg;
//sensor_msgs::Imu imu_msg;
//geometry_msgs::Vector3 vec3_msg;
//nav_msgs::Odometry odom_msg;

// Array representation of adafruit_bno055_offsets_t.
//uint16_t imu_calibration_array[11] = {0};

//#define MAX_OUT_CHARS 50
//char buffer[MAX_OUT_CHARS + 1];  //buffer used to format a line (+1 is for trailing 0)

//TODO:remove once Arduino IDE upgraded on RPi?
// Arduino Leonardo & Yun
//#define digitalPinToInterrupt(p)  ( (p) == 0 ? 2 : ((p) == 1 ? 3 : ((p) == 2 ? 1 : ((p) == 3 ? 0 : ((p) == 7 ? 4 : -1)))) ) 

//bool report_diagnostics = false;
//unsigned long last_diagnostic = 0;

//ros::Publisher diagnostics_publisher = ros::Publisher("diagnostics_relay", &string_msg);

void count_pan_changes(){
    /*
    CW  => 11,00,11,00,...
    CCW => 10,01,10,01,...
    */
    int a = digitalRead(PAN_MOTOR_ENCODER_A);
    int b = digitalRead(PAN_MOTOR_ENCODER_B);
    int direction = (a == b) ? +1 : -1;
    pan_controller.set_encoder_feedback(direction, 1);
    pan_controller.update();
}

void halt_all_activity(){
    pan_controller.set_power(false);
    pan_controller.stop();
    tilt_controller.stop();
}

// rostopic pub --once /head_arduino/pan/set std_msgs/Int16 90
// 0=origin/forward/center
// 180=backwards
// 360=origin/forward/center
void on_pan_angle_set(const std_msgs::Int16& msg) {
    pan_controller.set_target_angle(msg.data);
}
ros::Subscriber<std_msgs::Int16> on_pan_angle_set_sub("pan/set", &on_pan_angle_set);

// rostopic pub --once /head_arduino/tilt/set std_msgs/Int16 180
// 0=all the way down
// 90=level
// 180=all the way up
void on_tilt_angle_set(const std_msgs::Int16& msg) {
    tilt_controller.set_target_degrees(msg.data, true);
}
ros::Subscriber<std_msgs::Int16> on_tilt_angle_set_sub("tilt/set", &on_tilt_angle_set);

// rostopic pub --once /head_arduino/halt std_msgs/Empty
void on_halt(const std_msgs::Empty& msg) {
    halt_all_activity();
}
ros::Subscriber<std_msgs::Empty> on_halt_sub("halt", &on_halt);

void setup() {

    //last_diagnostic = millis();

    // Turn on power status light.
    //pinMode(STATUS_LED_PIN, OUTPUT);
    //digitalWrite(STATUS_LED_PIN, true);
    //digitalWrite(STATUS_LED_PIN, 0);

    //nh.getHardware()->setBaud(57600);
    nh.getHardware()->setBaud(115200); // loses connection?
    nh.initNode();

    // Register subscriptions.
    //nh.subscribe(toggle_led_sub);
    nh.subscribe(on_pan_angle_set_sub);
    nh.subscribe(on_tilt_angle_set_sub);
    nh.subscribe(on_halt_sub);

    // Register publishers.
    //nh.advertise(diagnostics_publisher);

    // Join I2C bus as Master with address #1
    //Wire.begin(1);
    //Wire.setTimeout(1000L);
    
    attachInterrupt(digitalPinToInterrupt(PAN_MOTOR_ENCODER_A), count_pan_changes, CHANGE);
    
    pan_controller.set_power(true);
    tilt_controller.set_power(true);

}

//long ftol(double v) {
    // Assumes 3 places of decimal precision.
    // Assumes the host interpreting this number will first divide by 1000.
//    return static_cast<long>(v*1000);
//}

//void send_diagnostics() {
    // Assumes you called `snprintf(buffer, MAX_OUT_CHARS, "key:value");` first
//    string_msg.data = buffer;
//    diagnostics_publisher.publish(&string_msg);
//}

void loop() {

    //report_diagnostics = false;
    //if (millis() - last_diagnostic >= DIAGNOSTIC_REPORT_FREQ_MS) {
        //last_diagnostic = millis();
        //report_diagnostics = true;
    //}
    // Track connection status with host.
    if (nh.connected()) {
        //digitalWrite(STATUS_LED_PIN, 1);
        connected = true;
    } else if (connected) {
        // If we just became disconnected from the host, then immediately halt all motors as a safety precaution.
        //digitalWrite(STATUS_LED_PIN, 0);
        halt_all_activity();
        delay(50);
        connected = false;
    }

    //force_sensors = false;

    // Update all sensors and controllers.
    pan_controller.update();
    tilt_controller.update();

    nh.spinOnce();
    delay(1);

}
