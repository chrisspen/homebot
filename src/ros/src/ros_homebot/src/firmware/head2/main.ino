/*
 * Copyright 2017 Chris Spencer
 */
// http://wiki.ros.org/rosserial_arduino/Tutorials/Hello%20World
//#include "Arduino.h"
//#define USE_USBCON
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
#include <std_msgs/Float32MultiArray.h>
//#include <std_msgs/Int32MultiArray.h>
//#include <std_msgs/Int16MultiArray.h>
//#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>
//#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/String.h>
//#include <geometry_msgs/Twist.h>
//#include <diagnostic_msgs/DiagnosticArray.h>
//#include <diagnostic_msgs/DiagnosticStatus.h>
//#include <diagnostic_msgs/KeyValue.h>
//#include <Wire.h>

//#include <SoftPWM.h>

#include "ArduinoPinout.h"
#include "StringUtils.h"
#include "PanController.h"
#include "TiltController.h"

//#define DIAGNOSTIC_STATUS_LENGTH 1
#define DIAGNOSTIC_REPORT_FREQ_MS 1000

long ftol(double v) {
    // Assumes 3 places of decimal precision.
    // Assumes the host interpreting this number will first divide by 1000.
    return static_cast<long>(v*1000);
}

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
bool force_sensors = false;

// Records when we last received a message from rosserial on the host.
// This is used to detect a lost connection to the host and perform emergency shutdown of the motors.
bool connected = false;

// http://wiki.ros.org/roscpp/Overview/Logging
// http://wiki.ros.org/rosserial/Overview/Logging
ros::NodeHandle nh;

//http://wiki.ros.org/std_msgs
//http://wiki.ros.org/common_msgs
std_msgs::Int16 int16_msg;
//std_msgs::Byte byte_msg;
//std_msgs::Bool bool_msg;
//std_msgs::Float32 float_msg;
std_msgs::String string_msg;
//std_msgs::Float32MultiArray float32ma_msg;
//std_msgs::UInt16MultiArray uint16ma_msg;
//sensor_msgs::Range range_msg;
//sensor_msgs::BatteryState battery_msg;
//sensor_msgs::Imu imu_msg;
//geometry_msgs::Vector3 vec3_msg;
//nav_msgs::Odometry odom_msg;

// Array representation of adafruit_bno055_offsets_t.
//uint16_t imu_calibration_array[11] = {0};

#define MAX_OUT_CHARS 50
char buffer[MAX_OUT_CHARS + 1];  //buffer used to format a line (+1 is for trailing 0)

//TODO:remove once Arduino IDE upgraded on RPi?
// Arduino Leonardo & Yun
//#define digitalPinToInterrupt(p)  ( (p) == 0 ? 2 : ((p) == 1 ? 3 : ((p) == 2 ? 1 : ((p) == 3 ? 0 : ((p) == 7 ? 4 : -1)))) )

bool report_diagnostics = false;
unsigned long last_diagnostic = 0;

ros::Publisher diagnostics_publisher = ros::Publisher("diagnostics_relay", &string_msg);

void on_pan_encoder_change() {
    /*
    CW  => 11,00,11,00,...
    CCW => 10,01,10,01,...
    */
    int a = digitalRead(PAN_MOTOR_ENCODER_A);
    int b = digitalRead(PAN_MOTOR_ENCODER_B);
    int direction = (a == b) ? +1 : -1;
    pan_controller.set_encoder_feedback(direction, 1);
    pan_controller.update();
    //nh.loginfo("pan encoder changed");
}

void on_pan_reference_change() {
    pan_controller.update_centermark();
    //nh.loginfo("pan ref changed");
}

void halt_all_activity() {
    pan_controller.set_power(false);
    pan_controller.stop();
    tilt_controller.stop();
}

// rostopic echo /head_arduino/pan_degrees
ros::Publisher pan_angle_publisher = ros::Publisher("pan_degrees", &int16_msg);

// rostopic echo /head_arduino/pan_error
ros::Publisher pan_error_publisher = ros::Publisher("pan_error", &int16_msg);

// rostopic echo /head_arduino/tilt_degrees
ros::Publisher tilt_angle_publisher = ros::Publisher("tilt_degrees", &int16_msg);

// rostopic pub --once /head_arduino/force_sensors std_msgs/Empty
void on_force_sensors(const std_msgs::Empty& msg) {
    force_sensors = true;
}
ros::Subscriber<std_msgs::Empty> on_force_sensors_sub("force_sensors", &on_force_sensors);

// rostopic pub --once /head_arduino/pan_calibrate std_msgs/Empty
void on_pan_angle_calibrate(const std_msgs::Empty& msg) {
    pan_controller.calibrate();
}
ros::Subscriber<std_msgs::Empty> on_pan_angle_calibrate_sub("pan_calibrate", &on_pan_angle_calibrate);

void on_torso_imu_euler_z(const std_msgs::Int16& msg) {
    //pan_controller.torso_imu_euler_z1 = msg.data;
    pan_controller.update_torso_z(msg.data);
    snprintf(buffer, MAX_OUT_CHARS, "received torso z: %d", msg.data);//TODO
    nh.loginfo(buffer);
}
ros::Subscriber<std_msgs::Int16> on_torso_imu_euler_z_sub("/torso_arduino/imu_euler_z", &on_torso_imu_euler_z);

// rostopic pub --once /head_arduino/pan_pid_set std_msgs/Float32MultiArray "{layout:{dim:[], data_offset: 0}, data:[0.95, 0.15, 0.1]}"
// rostopic pub --once /head_arduino/pan_pid_set std_msgs/Float32MultiArray "{layout:{dim:[], data_offset: 0}, data:[1, 0, 0]}"
void on_pan_pid_set(const std_msgs::Float32MultiArray& msg) {
    pan_controller.kp = msg.data[0];
    pan_controller.ki = msg.data[1];
    pan_controller.kd = msg.data[2];
    nh.loginfo("Pan PID parameters changed.");
    snprintf(buffer, MAX_OUT_CHARS, "kp: %ld", ftol(pan_controller.kp));
    nh.loginfo(buffer);
    snprintf(buffer, MAX_OUT_CHARS, "ki: %ld", ftol(pan_controller.ki));
    nh.loginfo(buffer);
    snprintf(buffer, MAX_OUT_CHARS, "kd: %ld", ftol(pan_controller.kd));
    nh.loginfo(buffer);
}
ros::Subscriber<std_msgs::Float32MultiArray> on_pan_pid_set_sub("pan_pid_set", &on_pan_pid_set);

// Tells the pan motor to turn head to a specific position as quickly as possible.
// rostopic pub --once /head_arduino/pan_set std_msgs/Int16 90
// 0=origin/forward/center
// 180=backwards
// 360=origin/forward/center
// 356=head facing true center
void on_pan_angle_set(const std_msgs::Int16& msg) {
    pan_controller.active = true;
    pan_controller.set_target_angle(msg.data);
    snprintf(buffer, MAX_OUT_CHARS, "Set pan angle: %d", msg.data);
    nh.loginfo(buffer);
}
ros::Subscriber<std_msgs::Int16> on_pan_angle_set_sub("pan_set", &on_pan_angle_set);

// Tells the pan motor to move head at a specific constant speed indefinitely.
// Speed is specified in degrees/sec.
// A negative value is counter-clockwise.
// A positive value is clockwise.
void on_pan_speed_set(const std_msgs::Int16& msg) {
    pan_controller.active = true;
    pan_controller.set_target_dps(msg.data);
    snprintf(buffer, MAX_OUT_CHARS, "Set pan speed degrees: %d", msg.data);
    nh.loginfo(buffer);
}
ros::Subscriber<std_msgs::Int16> on_pan_speed_set_sub("pan_speed_set", &on_pan_speed_set);

// rostopic pub --once /head_arduino/pan_freeze_set std_msgs/Bool 1
// Tells the pan motor to freeze its position relative to the torso.
// If the torso rotates, the head with counter-rotate to maintain its position.
void on_pan_freeze_set(const std_msgs::Bool& msg) {
    pan_controller.set_freeze(msg.data);
    snprintf(buffer, MAX_OUT_CHARS, "Pan angle freeze set: %d", msg.data);
    nh.loginfo(buffer);
}
ros::Subscriber<std_msgs::Bool> on_pan_freeze_set_sub("pan_freeze_set", &on_pan_freeze_set);

// rostopic pub --once /head_arduino/tilt_set std_msgs/Int16 90
// 0=all the way down
// 90=level (-/+ 30 is safe range)
// 180=all the way up
void on_tilt_angle_set(const std_msgs::Int16& msg) {
    tilt_controller.set_target_degrees(msg.data, true);
}
ros::Subscriber<std_msgs::Int16> on_tilt_angle_set_sub("tilt_set", &on_tilt_angle_set);

// rostopic pub --once /head_arduino/halt std_msgs/Empty
void on_halt(const std_msgs::Empty& msg) {
    halt_all_activity();
}
ros::Subscriber<std_msgs::Empty> on_halt_sub("halt", &on_halt);

void toggle_led() {
    digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}
// rostopic pub --once /head_arduino/toggle_led std_msgs/Empty
// rostopic pub --rate 1 /head_arduino/toggle_led std_msgs/Empty
void on_toggle_led(const std_msgs::Empty& msg) {
    nh.loginfo("LED toggled.");
    toggle_led();
}
ros::Subscriber<std_msgs::Empty> toggle_led_sub("toggle_led", &on_toggle_led);

// rostopic pub --once /head_arduino/linelaser_set std_msgs/Bool 1
// rostopic pub --rate 1 /head_arduino/linelaser_set std_msgs/Bool
void on_line_laser_set(const std_msgs::Bool& msg) {
    snprintf(buffer, MAX_OUT_CHARS, "linelaser_set:%d", msg.data);
    nh.loginfo(buffer);
    digitalWrite(LINE_LASER_SET, msg.data);
}
ros::Subscriber<std_msgs::Bool> on_line_laser_set_sub("linelaser_set", &on_line_laser_set);

// rostopic pub --once /head_arduino/ultrabright_set std_msgs/Int16 0
// rostopic pub --once /head_arduino/ultrabright_set std_msgs/Int16 254
void on_set_ultrabright(const std_msgs::Int16& msg) {
    // msg.data should be between [0-255]
    //SoftPWMSet(ULTRABRIGHT_LED_1, msg.data);
    //SoftPWMSet(ULTRABRIGHT_LED_2, msg.data);
    //SoftPWMSet(ULTRABRIGHT_LED_3, msg.data);
    digitalWrite(ULTRABRIGHT_LED_1, msg.data);
    //digitalWrite(ULTRABRIGHT_LED_2, msg.data);
    //digitalWrite(ULTRABRIGHT_LED_3, msg.data);
}
ros::Subscriber<std_msgs::Int16> set_ultrabright_sub("ultrabright_set", &on_set_ultrabright);

// rostopic pub --once /head_arduino/rgb_set std_msgs/UInt8MultiArray "{layout:{dim:[], data_offset: 0}, data:[1, 0, 0]}"
void on_set_rgb(const std_msgs::UInt8MultiArray& msg) {
    // msg.data should be between [0-1]
    digitalWrite(STATUS_LED_RED, msg.data[0]);
    digitalWrite(STATUS_LED_GREEN, msg.data[1]);
    digitalWrite(STATUS_LED_BLUE, msg.data[2]);
}
ros::Subscriber<std_msgs::UInt8MultiArray> set_rgb_sub("rgb_set", &on_set_rgb);

void setup() {

    //last_diagnostic = millis();

    // Turn on power status light.
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    //nh.getHardware()->setBaud(57600);
    nh.getHardware()->setBaud(115200);
    nh.initNode();

    // Register subscriptions.
    // Note, this will require editing ros.h to support more than 10 subscribers.
    // Edit with: nano ./lib/ros_lib/ros.h
    // typedef NodeHandle_<ArduinoHardware, 20, 20, 512, 512> NodeHandle;
    // TODO: fix so this hacky manual edit isn't needed?
    nh.subscribe(toggle_led_sub);
    nh.subscribe(set_ultrabright_sub);
    nh.subscribe(set_rgb_sub);
    nh.subscribe(on_force_sensors_sub);
    nh.subscribe(on_line_laser_set_sub);
    nh.subscribe(on_pan_angle_set_sub);
    nh.subscribe(on_tilt_angle_set_sub);
    nh.subscribe(on_halt_sub);
    nh.subscribe(on_pan_angle_calibrate_sub);
    nh.subscribe(on_pan_speed_set_sub);
    nh.subscribe(on_pan_freeze_set_sub);
    nh.subscribe(on_pan_pid_set_sub);
    nh.subscribe(on_torso_imu_euler_z_sub);

    // Register publishers.
    nh.advertise(diagnostics_publisher);
    nh.advertise(pan_angle_publisher);
    nh.advertise(tilt_angle_publisher);
    nh.advertise(pan_error_publisher);

    attachInterrupt(digitalPinToInterrupt(PAN_MOTOR_ENCODER_A), on_pan_encoder_change, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PAN_MOTOR_POSITION_REF), on_pan_reference_change, CHANGE);
    //pan_controller.set_power(true);
    //tilt_controller.set_power(true);

    // Initialize ultrabright dimmers and set default to off.
    //SoftPWMBegin();
    //SoftPWMSet(ULTRABRIGHT_LED_1, 0);
    //SoftPWMSetFadeTime(ULTRABRIGHT_LED_1, 1000, 1000);
    //SoftPWMSet(ULTRABRIGHT_LED_2, 0);
    //SoftPWMSetFadeTime(ULTRABRIGHT_LED_2, 1000, 1000);
    //SoftPWMSet(ULTRABRIGHT_LED_3, 0);
    //SoftPWMSetFadeTime(ULTRABRIGHT_LED_2, 1000, 1000);
    pinMode(ULTRABRIGHT_LED_1, OUTPUT);
    //pinMode(ULTRABRIGHT_LED_2, OUTPUT);
    //pinMode(ULTRABRIGHT_LED_3, OUTPUT);
    digitalWrite(ULTRABRIGHT_LED_1, 0);
    //digitalWrite(ULTRABRIGHT_LED_2, 0);
    //igitalWrite(ULTRABRIGHT_LED_3, 0);

    pinMode(STATUS_LED_RED, OUTPUT);
    pinMode(STATUS_LED_GREEN, OUTPUT);
    pinMode(STATUS_LED_BLUE, OUTPUT);
    digitalWrite(STATUS_LED_RED, 0);
    digitalWrite(STATUS_LED_GREEN, 0);
    digitalWrite(STATUS_LED_BLUE, 0);

    pinMode(LINE_LASER_SET, OUTPUT);
    digitalWrite(LINE_LASER_SET, 0);
}

//long ftol(double v) {
    // Assumes 3 places of decimal precision.
    // Assumes the host interpreting this number will first divide by 1000.
//    return static_cast<long>(v*1000);
//}

void send_diagnostics() {
    // Assumes you called `snprintf(buffer, MAX_OUT_CHARS, "key:value");` first
    string_msg.data = buffer;
    diagnostics_publisher.publish(&string_msg);
}

void loop() {

    report_diagnostics = false;
    if (millis() - last_diagnostic >= DIAGNOSTIC_REPORT_FREQ_MS) {
        last_diagnostic = millis();
        report_diagnostics = true;
        
        //snprintf(buffer, MAX_OUT_CHARS, "pan speed:%d", pan_controller.get_speed());
        //nh.loginfo(buffer);
        //snprintf(buffer, MAX_OUT_CHARS, "pan actual angle:%ld", ftol(pan_controller.actual_angle.get_latest()));
        //nh.loginfo(buffer);
        //snprintf(buffer, MAX_OUT_CHARS, "pan target angle:%ld", ftol(pan_controller.get_target_angle()));
        //nh.loginfo(buffer);
        //snprintf(buffer, MAX_OUT_CHARS, "pan pos_err:%d", pan_controller.pos_err);
        //nh.loginfo(buffer);
        //snprintf(buffer, MAX_OUT_CHARS, "pan td_err:%d", pan_controller.td_err);
        //nh.loginfo(buffer);
        //snprintf(buffer, MAX_OUT_CHARS, "pan sum_err:%d", pan_controller.sum_err);
        //nh.loginfo(buffer);
        
        //snprintf(buffer, MAX_OUT_CHARS, "dps_target:%d", (int)pan_controller._dps);
        //nh.loginfo(buffer);
        //snprintf(buffer, MAX_OUT_CHARS, "dps_effective:%d", (int)pan_controller._dps_effective);
        //nh.loginfo(buffer);
        //snprintf(buffer, MAX_OUT_CHARS, "speed:%d", pan_controller._speed);
        //nh.loginfo(buffer);
        
        //snprintf(buffer, MAX_OUT_CHARS, "torso_imu_euler_z0:%d", (int)pan_controller.torso_imu_euler_z0);
        //nh.loginfo(buffer);
        //snprintf(buffer, MAX_OUT_CHARS, "torso_imu_euler_z1:%d", (int)pan_controller.torso_imu_euler_z1);
        //nh.loginfo(buffer);
        //snprintf(buffer, MAX_OUT_CHARS, "freeze:%d", (int)pan_controller.get_freeze());
        //nh.loginfo(buffer);
        //snprintf(buffer, MAX_OUT_CHARS, "seeking:%d", (int)pan_controller.get_seeking());
        //nh.loginfo(buffer);
    }

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

    // Output sensor readings.
    if (pan_controller.actual_angle.get_and_clear_changed() || force_sensors || report_diagnostics) {
        int16_msg.data = pan_controller.actual_angle.get_latest();
        pan_angle_publisher.publish(&int16_msg);
    }
    if (tilt_controller.actual_degrees.get_and_clear_changed() || force_sensors || report_diagnostics) {
        int16_msg.data = tilt_controller.actual_degrees.get_latest();
        tilt_angle_publisher.publish(&int16_msg);
    }
    
    // Output any pending pan angle error reports.
    if(!pan_controller.is_seeking() && pan_controller.error_pending){
        pan_controller.error_pending = false;
        int16_msg.data = pan_controller.get_angle_error();
        pan_error_publisher.publish(&int16_msg);
    }

    force_sensors = false;

    // Update all sensors and controllers.
    pan_controller.update();
    tilt_controller.update();

    nh.spinOnce();
    delay(1);

}
