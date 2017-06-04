/*
 * Copyright 2017 Chris Spencer
 */
// http://wiki.ros.org/rosserial_arduino/Tutorials/Hello%20World
//#include "Arduino.h"
#include <ros.h>
#include <math.h>
#include <stdint.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
//#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
//#include <diagnostic_msgs/KeyValue.h>
#include <Wire.h>

#include "ArduinoPinout.h"
#include "I2CAddresses.h"
#include "BooleanSensor.h"
#include "BatteryVoltageSensor.h"
#include "MotionController.h"
#include "UltrasonicSensor.h"
#include "AccelGyroSensor.h"
#include "PowerController.h"
#include "ArduinoTemperatureSensor.h"
#include "AnalogSensor.h"
#include "OdometryTracker.h"

//#define PING_TIMEOUT_MS 10000

#define DIAGNOSTIC_STATUS_LENGTH 1
#define DIAGNOSTIC_REPORT_FREQ_MS 1000

char base_link[] = "/base_link";
const char * const ultrasonic_links[] = { "/base_link/ultrasonic0", "/base_link/ultrasonic1", "/base_link/ultrasonic2" };
char odom_link[] = "/odom";

// Convert the BNO055's calibration code [0-3] to a diagnostics status.
int calib_to_status[] = {
    diagnostic_msgs::DiagnosticStatus::ERROR, //2, raw 0=uncalibrated
    diagnostic_msgs::DiagnosticStatus::WARN,  //1, raw 1=partial
    diagnostic_msgs::DiagnosticStatus::WARN,  //1, raw 2=almost
    diagnostic_msgs::DiagnosticStatus::OK     //0, raw 3=completely calibrated
};

AccelGyroSensor ag_sensor = AccelGyroSensor();

MotionController motion_controller;

OdometryTracker odometry_tracker = OdometryTracker();

geometry_msgs::TransformStamped ts;
tf::TransformBroadcaster tf_broadcaster;
//tf::Transform transform;
//tf::TransformBroadcaster br;
//tf::Transform transform;

// If true, ultrasonic sensors will be read and reported.
bool ultrasonics_enabled = false;

// Space each reading to prevent echo from interfering with sequential readings.
int ultrasonics_spacing = 10;

// If set to true, all motors will be stopped immediately.
bool halt = false;

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
std_msgs::Byte byte_msg;
std_msgs::Bool bool_msg;
std_msgs::Float32 float_msg;
std_msgs::String string_msg;
std_msgs::UInt16MultiArray uint16ma_msg;
sensor_msgs::Range range_msg;
sensor_msgs::BatteryState battery_msg;
sensor_msgs::Imu imu_msg;
geometry_msgs::Vector3 vec3_msg;
nav_msgs::Odometry odom_msg;

// Array representation of adafruit_bno055_offsets_t.
//uint16_t imu_calibration_array[11] = {0};
adafruit_bno055_offsets_t imu_calib_struct;

#define MAX_OUT_CHARS 255
char buffer[MAX_OUT_CHARS + 1];  //buffer used to format a line (+1 is for trailing 0)

bool battery_changed = false;

unsigned long last_debug = 0;

bool report_diagnostics = false;
unsigned long last_diagnostic = 0;

unsigned long a_encoder_last_change = 0;
unsigned long b_encoder_last_change = 0;

//diagnostic_msgs::DiagnosticArray dia_array;
//diagnostic_msgs::DiagnosticStatus robot_status;
//diagnostic_msgs::KeyValue status_keyvalue;

/*
 * Begin sensor definitions.
 */

BatteryVoltageSensor battery_voltage_sensor = BatteryVoltageSensor(
    // The battery consists of 3 * 4.2V lipo cells.
    12.6,
    2000000, // ohms
    1000000, // ohms
    BATTERY_VOLTAGE_PIN,
    // dead_ratio, The battery is considered dead when 80% discharged.
    0.8,
    // voltage_offset, 10.9 measured by the Arduino's ADC is really 11.9 as measured by a good multimeter.
    2
);

AnalogSensor external_power_sensors[2] = {
    // EP1
    // The pin measuring immediately after the external connector but before the reed switch.
    // Reads high when the external power plug is connected and external power is present.
    AnalogSensor(EXTERNAL_POWER_SENSE_1_PIN),
    // EP2
    // The pin measuring behind the reed switch.
    // Reads high when the external power plug is connected, regardless of whether or not
    // external power is present.
    // This lets the robot know if it's connected to a dead recharge station.
    AnalogSensor(EXTERNAL_POWER_SENSE_2_PIN)
    // We're only fully docked and charging when both EP1 and EP2 are high.
};

//ArduinoTemperatureSensor arduino_temperature_sensor = ArduinoTemperatureSensor();

PowerController power_controller = PowerController();

BooleanSensor edge_sensors[3] = {
    BooleanSensor(EDGE_L_PIN),
    BooleanSensor(EDGE_M_PIN),
    BooleanSensor(EDGE_R_PIN)
};

//DEPRECATED
BooleanSensor bumper_sensors[3] = {
    BooleanSensor(BUMPER_L_PIN),
    BooleanSensor(BUMPER_M_PIN),
    BooleanSensor(BUMPER_R_PIN)
};

UltrasonicSensor ultrasonic_sensors[3] = {
    UltrasonicSensor(SONIC_L_PIN),
    UltrasonicSensor(SONIC_M_PIN),
    UltrasonicSensor(SONIC_R_PIN)
};

BooleanSensor power_button_sensor = BooleanSensor(SIGNAL_BUTTON_PIN, true);

/*
 * End sensor definitions.
 */

/*
 * Begin publisher definitions.
 */

//ros::Publisher battery_voltage_publisher = ros::Publisher("power/voltage", &float_msg);
//ros::Publisher battery_charge_publisher = ros::Publisher("power/charge", &float_msg);

//ros::Publisher diagnostics_publisher = ros::Publisher("diagnostics", &dia_array);
//nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);

// rostopic echo /torso_arduino/battery
ros::Publisher battery_state_publisher = ros::Publisher("battery", &battery_msg);

ros::Publisher external_power_publishers[2] = {
    ros::Publisher("power/external/0", &bool_msg), // external power voltage present
    ros::Publisher("power/external/1", &bool_msg) // external power magnet present
};

ros::Publisher edge_publishers[3] = {
    ros::Publisher("edge/0", &bool_msg),
    ros::Publisher("edge/1", &bool_msg),
    ros::Publisher("edge/2", &bool_msg)
};

//TODO:enable once physical switches are improved, otherwise this is worthless
//ros::Publisher bumper_publishers[3] = {
//    ros::Publisher("bumper/0", &bool_msg),
//    ros::Publisher("bumper/1", &bool_msg),
//    ros::Publisher("bumper/2", &bool_msg)
//};

ros::Publisher ultrasonic_publishers[3] = {
//    ros::Publisher("ultrasonic/0", &int16_msg),
//    ros::Publisher("ultrasonic/1", &int16_msg),
//    ros::Publisher("ultrasonic/2", &int16_msg)
    ros::Publisher("ultrasonic/0", &range_msg),
    ros::Publisher("ultrasonic/1", &range_msg),
    ros::Publisher("ultrasonic/2", &range_msg)
};

ros::Publisher power_button_publisher = ros::Publisher("power/button", &bool_msg);

ros::Publisher motor_a_encoder_publisher = ros::Publisher("motor/encoder/a", &int16_msg);
ros::Publisher motor_b_encoder_publisher = ros::Publisher("motor/encoder/b", &int16_msg);
ros::Publisher motor_error_publisher = ros::Publisher("motor/error", &byte_msg);

// https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/device-calibration
// Directly from the bno.getCalibration(), 0=not calibrated, 3=fully calibrated
// The reason is that system cal '0' in NDOF mode means that the device has not yet found the 'north pole',
// and orientation values will be off  The heading will jump to an absolute value once the BNO finds magnetic north
// (the system calibration status jumps to 1 or higher).
ros::Publisher imu_calibration_sys_publisher = ros::Publisher("imu/calibration/sys", &int16_msg);

// To calibrate, the device must be standing still in any position.
ros::Publisher imu_calibration_gyr_publisher = ros::Publisher("imu/calibration/gyr", &int16_msg);

// This seems to take forever to calibrate, no matter how long you wait or move the sensor.
// The BNO055 must be placed in 6 standing positions for +X, -X, +Y, -Y, +Z and -Z.
// This is the most onerous sensor to calibrate, but the best solution to generate the calibration data
// is to find a block of wood or similar object, and place the sensor on each of the 6 'faces' of the block,
// which will help to maintain sensor alignment during the calibration process.
// You should still be able to get reasonable quality data from the BNO055, however, even if the accelerometer
// isn't entirely or perfectly calibrated.
// This also isn't necessary for the sys output to read 3, or fully calibrated.
ros::Publisher imu_calibration_acc_publisher = ros::Publisher("imu/calibration/acc", &int16_msg);

// To calibrate, the sensor must be moved in a 'figure 8' motion ideally, or at least some minimal movement.
// Note, mag will remain at 0 until it's moved a little, then it will switch to 1 and then 2, and then after a few more seconds, fully to 3.
// The lesson here is that simply waiting for the IMU to fully calibrate won't work. The robot needs to move itself around a little.
ros::Publisher imu_calibration_mag_publisher = ros::Publisher("imu/calibration/mag", &int16_msg);

// When the bno's sys calibration flag reads 3, or fully calibrated, this publishes the 11 ints
// representing the calibration value to store in an adafruit_bno055_offsets_t.
ros::Publisher imu_calibration_save_publisher = ros::Publisher("imu/calibration/save", &uint16ma_msg);

//ros::Publisher imu_publisher = ros::Publisher("imu", &imu_msg);
//ros::Publisher imu_euler_publisher = ros::Publisher("imu/euler", &vec3_msg);
//ros::Publisher imu_accel_publisher = ros::Publisher("imu/accel", &vec3_msg);
ros::Publisher imu_publisher = ros::Publisher("imu_relay", &string_msg);

// rostopic echo /torso_arduino/odometry_relay
ros::Publisher odometry_publisher = ros::Publisher("odometry_relay", &string_msg);

ros::Publisher diagnostics_publisher = ros::Publisher("diagnostics_relay", &string_msg);

/*
 * End publisher definitions.
 */

uint8_t get_battery_supply_status() {
    if (external_power_sensors[0].get_bool() && external_power_sensors[1].get_bool()) {
        // If the external power connection is present and reading voltage, then we're either charging or fully charged.
        if (battery_voltage_sensor.get_charge_ratio_int() >= 95) {
            return sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_FULL;
        } else {
            return sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
        }
    } else if (battery_voltage_sensor.get_charge_ratio_int() <= 1 && !external_power_sensors[1].get_bool()) {
        // If the external power connector is reading not present but we're also reading near-zero battery voltage,
        // but we're still computing, then that means we're on external power and the battery has been physically removed.
        return sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
    } else {
        // Otherwise, we're on battery power and discharging.
        return sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    }
}

bool is_battery_present() {
    if (get_battery_supply_status() == sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING) {
        return false;
    }
    return true;
}

void toggle_led() {
    digitalWrite(STATUS_LED_PIN, HIGH-digitalRead(STATUS_LED_PIN));   // blink the led
}

void stop_motors() {
    nh.loginfo("Motors halted!");
    //motion_controller.stop();
    motion_controller.set(0, 0);
    //TODO:publish motor state?
}

/*
 * Begin subscribers.
 */

// rostopic pub --once /torso_arduino/ultrasonics/enabled std_msgs/Bool 1
// rostopic pub --once /torso_arduino/ultrasonics/enabled std_msgs/Bool 0
void on_ultrasonics_enabled(const std_msgs::Bool& msg) {
    if (msg.data) {
        nh.loginfo("Ultrasonics enabled.");
    } else {
        nh.loginfo("Ultrasonics disabled.");
    }
    ultrasonics_enabled = msg.data;
}
ros::Subscriber<std_msgs::Bool> ultrasonics_enabled_sub("ultrasonics/enabled", &on_ultrasonics_enabled);

// rostopic pub --once /torso_arduino/halt std_msgs/Empty
void on_halt(const std_msgs::Empty& msg) {
    stop_motors();
}
ros::Subscriber<std_msgs::Empty> halt_sub("halt", &on_halt);

// rostopic pub --once /torso_arduino/force_sensors std_msgs/Empty
void on_force_sensors(const std_msgs::Empty& msg) {
    force_sensors = true;
}
ros::Subscriber<std_msgs::Empty> force_sensors_sub("force_sensors", &on_force_sensors);

// rostopic pub --once /torso_arduino/toggle_led std_msgs/Empty
void on_toggle_led(const std_msgs::Empty& msg) {
    nh.loginfo("LED toggled.");
    toggle_led();
}
ros::Subscriber<std_msgs::Empty> toggle_led_sub("toggle_led", &on_toggle_led);

// rostopic pub -1 /torso_arduino/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
void on_cmd_vel(const geometry_msgs::Twist& msg) {
    nh.loginfo("Received cmd_vel.");
    motion_controller.set_cmd_vel(msg.linear.x, msg.angular.z);
}
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &on_cmd_vel);

// rostopic pub --once /torso_arduino/motor/speed std_msgs/Int16MultiArray "{layout:{dim:[], data_offset: 0}, data:[64, 64]}"
// Note, if you get a connection error, that probably means main battery power was not on when you connected to the Arduino,
// and so the ComMotion didn't initialize properly.
// Fix this by turning on main battery power and then resetting the Arduino.
void on_motor_speed(const std_msgs::Int16MultiArray& msg) {
    nh.loginfo("Setting motor speed...");
    //toggle_led();
    if (motion_controller.connection_error) {
        nh.loginfo("Unable to set motor speed due to connection error.");
    } else {
        //set_motor_speeds(0, 0, msg.data[0], msg.data[1]);
        motion_controller.set(msg.data[0], msg.data[1]);
        nh.loginfo("Motor speed set.");
    }
}
ros::Subscriber<std_msgs::Int16MultiArray> motor_speed_sub("motor/speed", &on_motor_speed);

// rostopic pub --once /torso_arduino/imu/calibration/load std_msgs/UInt16MultiArray
// "{layout:{dim:[], data_offset: 0}, data:[0, 0, 0, 65534, 65534, 0, 65438, 65370, 268, 1000, 750]}"
void on_imu_calibration_load(const std_msgs::UInt16MultiArray& msg) {
    nh.loginfo("Loading IMU calibration...");
    adafruit_bno055_offsets_t newCalib;
    newCalib.accel_offset_x = msg.data[0];
    newCalib.accel_offset_y = msg.data[1];
    newCalib.accel_offset_z = msg.data[2];
    newCalib.gyro_offset_x = msg.data[3];
    newCalib.gyro_offset_y = msg.data[4];
    newCalib.gyro_offset_z = msg.data[5];
    newCalib.mag_offset_x = msg.data[6];
    newCalib.mag_offset_y = msg.data[7];
    newCalib.mag_offset_z = msg.data[8];
    newCalib.accel_radius = msg.data[9];
    newCalib.mag_radius = msg.data[10];
    ag_sensor.bno.setSensorOffsets(newCalib);
    nh.loginfo("IMU calibration loaded!");
}
ros::Subscriber<std_msgs::UInt16MultiArray> imu_calibration_load_sub("imu/calibration/load", &on_imu_calibration_load);

//            case ID_TWIST:
//                motion_controller.set_movement(
//                    stf(packet.get_arg(0)),
//                    stf(packet.get_arg(1)),
//                    stf(packet.get_arg(2)),
//                    packet.get_arg(3).toInt()
//                );
//                ack = true;
//                break;
//            case ID_MOTOR_ACCEL:
//                motion_controller.set_acceleration(packet.get_arg(0).toInt());
//                ack = true;
//                break;

//TODO:http://docs.ros.org/api/sensor_msgs/html/msg/BatteryState.html

/*
 * End subscribers.
 */
/*
static geometry_msgs::Quaternion createQuaternionFromRPY(double roll, double pitch, double yaw) {
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_Code
    // http://docs.ros.org/api/geometry_msgs/html/msg/Quaternion.html
    geometry_msgs::Quaternion q;
    double t0 = cos(yaw * 0.5);
    double t1 = sin(yaw * 0.5);
    double t2 = cos(roll * 0.5);
    double t3 = sin(roll * 0.5);
    double t4 = cos(pitch * 0.5);
    double t5 = sin(pitch * 0.5);
    q.w = t0 * t2 * t4 + t1 * t3 * t5;
    q.x = t0 * t3 * t4 - t1 * t2 * t5;
    q.y = t0 * t2 * t5 + t1 * t3 * t4;
    q.z = t1 * t2 * t4 - t0 * t3 * t5;
    return q;
}
*/

void setup() {

    // Initialize diagnostic status array.
    //http://docs.ros.org/diamondback/api/rosserial_arduino/html/classdiagnostic__msgs_1_1DiagnosticArray.html
    //dia_array.status = (diagnostic_msgs::DiagnosticStatus*)malloc(DIAGNOSTIC_STATUS_LENGTH * sizeof(diagnostic_msgs::DiagnosticStatus));
    //dia_array.status_length = DIAGNOSTIC_STATUS_LENGTH;
    //http://docs.ros.org/hydro/api/ric_mc/html/classdiagnostic__msgs_1_1DiagnosticStatus.html
    //robot_status.values = (diagnostic_msgs::KeyValue*)malloc(DIAGNOSTIC_STATUS_LENGTH * sizeof(diagnostic_msgs::KeyValue));
    //robot_status.values_length = DIAGNOSTIC_STATUS_LENGTH;
    //dia_array.status[0].values = (diagnostic_msgs::KeyValue*)malloc(DIAGNOSTIC_STATUS_LENGTH * sizeof(diagnostic_msgs::KeyValue));
    //dia_array.status[0].values_length = DIAGNOSTIC_STATUS_LENGTH;

    // http://answers.ros.org/question/10988/use-multiarray-in-rosserial/
    //uint16ma_msg.data = (uint16_t *)malloc(sizeof(uint16_t)*11);
    uint16ma_msg.data = reinterpret_cast<uint16_t *>(malloc(sizeof(uint16_t)*11));
    uint16ma_msg.data_length = 11;

    last_diagnostic = millis();

    // Turn on power status light.
    pinMode(STATUS_LED_PIN, OUTPUT);
    //digitalWrite(STATUS_LED_PIN, true);
    digitalWrite(STATUS_LED_PIN, 0);

//    pinMode(EXTERNAL_POWER_SENSE_1_PIN, INPUT);
//    pinMode(EXTERNAL_POWER_SENSE_2_PIN, INPUT);

    //nh.getHardware()->setBaud(57600);
    nh.getHardware()->setBaud(115200); // loses connection?
    nh.initNode();

    // Register subscriptions.
    nh.subscribe(toggle_led_sub);
    nh.subscribe(force_sensors_sub);
    nh.subscribe(halt_sub);
    nh.subscribe(ultrasonics_enabled_sub);
    nh.subscribe(motor_speed_sub);
    nh.subscribe(cmd_vel_sub);
    nh.subscribe(imu_calibration_load_sub);

    // Register publishers.
    //nh.advertise(battery_voltage_publisher);
    //nh.advertise(battery_charge_publisher);
    nh.advertise(battery_state_publisher);
    nh.advertise(power_button_publisher);
    for (int i = 0; i < 3; i++) {
        nh.advertise(edge_publishers[i]);
    }
//    for (int i = 0; i < 3; i++) {
//        nh.advertise(bumper_publishers[i]);
//    }
    for (int i = 0; i < 3; i++) {
        nh.advertise(ultrasonic_publishers[i]);
    }
    for (int i = 0; i < 2; i++) {
        nh.advertise(external_power_publishers[i]);
    }
    nh.advertise(motor_a_encoder_publisher);
    nh.advertise(motor_b_encoder_publisher);
    nh.advertise(motor_error_publisher);
    nh.advertise(imu_calibration_sys_publisher);
    nh.advertise(imu_calibration_gyr_publisher);
    nh.advertise(imu_calibration_acc_publisher);
    nh.advertise(imu_calibration_mag_publisher);
    nh.advertise(imu_calibration_save_publisher);
    nh.advertise(imu_publisher);
    nh.advertise(odometry_publisher);
    nh.advertise(diagnostics_publisher);

    // Join I2C bus as Master with address #1
    Wire.begin(1);
    Wire.setTimeout(1000L);

    // We initialize here so we can be sure it's done after the I2C initialization.
    motion_controller = MotionController();
    motion_controller.init();
    motion_controller.stop();

    ag_sensor.init();

}

long ftol(double v) {
    // Assumes 3 places of decimal precision.
    // Assumes the host interpreting this number will first divide by 1000.
    return static_cast<long>(v*1000);
}

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
    }

    //robot_status.name = "Robot";
    //robot_status.level = diagnostic_msgs::DiagnosticStatus::OK;
    //robot_status.message = "Everything seem to be ok.";
    //status_keyvalue.key = "A";
    //status_keyvalue.value = "B";
    //robot_status.values[0] = status_keyvalue;
    //robot_status.values.push_back(emergency);
    //robot_status.values->push_back(emergency);
    //dia_array.status.push_back(robot_status);
    //dia_array.status->push_back(robot_status);
    //dia_array.status[0] = robot_status;
    //dia_array.status[0].name = "R";
    //dia_array.status[0].level = diagnostic_msgs::DiagnosticStatus::OK;
    //dia_array.status[0].message = "E";
    //dia_array.status[0].values[0].key = "A";
    //dia_array.status[0].values[0].value = "B";
    //diagnostics_publisher.publish(&dia_array);
    //snprintf(buffer, MAX_OUT_CHARS, "hello");
    //string_msg.data = buffer;
    //diagnostics_publisher.publish(&string_msg);

//    if(millis() - last_debug > 1000){
//        last_debug = millis();
//        //snprintf(buffer, MAX_OUT_CHARS, "EXTERNAL_POWER_SENSE_1_PIN: %d", analogRead(EXTERNAL_POWER_SENSE_1_PIN));
//        snprintf(buffer, MAX_OUT_CHARS, "external voltage present? (EP1): %d", external_power_sensors[0].get_bool());
//        nh.loginfo(buffer);
//        //snprintf(buffer, MAX_OUT_CHARS, "EXTERNAL_POWER_SENSE_2_PIN: %d", analogRead(EXTERNAL_POWER_SENSE_2_PIN));
//        snprintf(buffer, MAX_OUT_CHARS, "external power plug attached? (EP2): %d", external_power_sensors[1].get_bool());
//        nh.loginfo(buffer);
//    }

    // Track connection status with host.
    if (nh.connected()) {
        digitalWrite(STATUS_LED_PIN, 1);
        connected = true;
    } else if (connected) {
        // If we just became disconnected from the host, then immediately halt all motors as a safety precaution.
        digitalWrite(STATUS_LED_PIN, 0);
        motion_controller.stop();
        delay(50);
        connected = false;
    }

    // Battery sensor.
    battery_voltage_sensor.update();
    if (battery_voltage_sensor.get_and_clear_changed() || force_sensors) {
        battery_changed = true;
    }

    // External power sensors.
    for (int i = 0; i < 2; i++) {
        external_power_sensors[i].update();
        if (external_power_sensors[i].get_and_clear_changed() || force_sensors) {
            bool_msg.data = external_power_sensors[i].get_bool();
            external_power_publishers[i].publish(&bool_msg);
        }
    }
    if (report_diagnostics) {
        // OK if external power is unplugged or plugged in and we detect a voltage.
        snprintf(buffer, MAX_OUT_CHARS, "ep.voltage.present:%d:%d",
            !(!external_power_sensors[1].get_bool() || (external_power_sensors[1].get_bool() && external_power_sensors[0].get_bool())),
            external_power_sensors[0].get_bool());
        send_diagnostics();

        snprintf(buffer, MAX_OUT_CHARS, "ep.magnet.present:%d:%d", diagnostic_msgs::DiagnosticStatus::OK, external_power_sensors[1].get_bool());
        send_diagnostics();
    }

    // Edge, bumper and ultrasonic sensors.
    for (int i = 0; i < 3; i++) {
        edge_sensors[i].update();
        if (edge_sensors[i].get_and_clear_changed() || force_sensors) {
            bool_msg.data = edge_sensors[i].value.get();
            edge_publishers[i].publish(&bool_msg);
        }
        if (report_diagnostics) {
            snprintf(buffer, MAX_OUT_CHARS, "edge.%d:%d:%d", i, diagnostic_msgs::DiagnosticStatus::OK, edge_sensors[i].value.get_latest());
            send_diagnostics();
        }
        // CS 2017.6.3 Disabled because these have a poor mechanical design, are unreliable, and are largely unnecessary with the presence of ultrasonics.
//        bumper_sensors[i].update();
//        if (bumper_sensors[i].get_and_clear_changed() || force_sensors) {
//            bool_msg.data = bumper_sensors[i].value.get();
//            bumper_publishers[i].publish(&bool_msg);
//        }
        if (ultrasonics_enabled) {
            ultrasonic_sensors[i].update();
            if (ultrasonic_sensors[i].get_and_clear_changed() || force_sensors) {
                // http://docs.ros.org/jade/api/sensor_msgs/html/msg/Range.html
                // http://wiki.ros.org/rosserial_arduino/Tutorials/Time%20and%20TF
                range_msg.header.stamp = nh.now();
                //snprintf(buffer, MAX_OUT_CHARS, "/base_link/ultrasonic%d", i);
                //range_msg.header.frame_id = buffer;
                range_msg.header.frame_id = ultrasonic_links[i];
                range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
                range_msg.field_of_view = 0.523599; // 30 degrees
                range_msg.min_range = 0.03; // meters
                range_msg.max_range = 4; // meters
                range_msg.range = ultrasonic_sensors[i].distance.get()/100.0; // meters
                ultrasonic_publishers[i].publish(&range_msg);
            }
            if (report_diagnostics) {
                snprintf(buffer, MAX_OUT_CHARS, "ultrasonic.%d.cm:%d:%d", i,
                    diagnostic_msgs::DiagnosticStatus::OK, static_cast<int>(ultrasonic_sensors[i].distance.get_latest()));
                send_diagnostics();
            }
            delay(ultrasonics_spacing);
        } else {
            if (report_diagnostics) {
                snprintf(buffer, MAX_OUT_CHARS, "ultrasonic.%d.cm:%d:disabled", i, diagnostic_msgs::DiagnosticStatus::OK);
                send_diagnostics();
            }
        }
    }

    // Temperature.
    // CS 2017.2.1 Disabled because not supported by the chip in the Arduino Uno*Pro.
    // If necessary, we could replace this with the temperature output from the IMU.
    // However, there's also a temp sensor in the head Leonardo, so this isn't strictly needed.
//    arduino_temperature_sensor.update();
//    if (arduino_temperature_sensor.get_and_clear_changed() || force_sensors) {
//        float_msg.data = arduino_temperature_sensor.temperature.get();
//        arduino_temperature_publisher.publish(&float_msg);
//    }

    // Motion controller.
    motion_controller.update();
    if (motion_controller.a_encoder.get_and_clear_changed() || motion_controller.b_encoder.get_and_clear_changed() || force_sensors) {
        a_encoder_last_change = millis();
        int16_msg.data = motion_controller.a_encoder.get_latest();
        motor_a_encoder_publisher.publish(&int16_msg);
        odometry_tracker.update_left(motion_controller.a_encoder.get_latest());
    }
    if (motion_controller.a_encoder.get_and_clear_changed() || motion_controller.b_encoder.get_and_clear_changed() || force_sensors) {
        b_encoder_last_change = millis();
        int16_msg.data = motion_controller.b_encoder.get_latest();
        motor_b_encoder_publisher.publish(&int16_msg);
        odometry_tracker.update_right(motion_controller.b_encoder.get_latest());
    }
    if (motion_controller.eflag.get_and_clear_changed() || force_sensors) {
        byte_msg.data = motion_controller.eflag.get_latest();
        motor_error_publisher.publish(&byte_msg);
    }
    if (report_diagnostics) {
        snprintf(buffer, MAX_OUT_CHARS, "motor.connection:%d", (motion_controller.eflag.get() & COMMOTION_ERROR_DISCONNECT));
        send_diagnostics();

        // CS 2017.6.3 The first two motor ports are unused, so we don't care about their current readings.
        //snprintf(buffer, MAX_OUT_CHARS, "motor.1.current.limit:%d", (motion_controller.eflag.get() & COMMOTION_ERROR_M1_MAXCURRENT));
        //send_diagnostics();
        //snprintf(buffer, MAX_OUT_CHARS, "motor.2.current.limit:%d", (motion_controller.eflag.get() & COMMOTION_ERROR_M2_MAXCURRENT));
        //send_diagnostics();

        //M3=right
        snprintf(buffer, MAX_OUT_CHARS, "motor.right.current.limit:%d", (motion_controller.eflag.get() & COMMOTION_ERROR_M3_MAXCURRENT));
        send_diagnostics();
        //M4=left
        snprintf(buffer, MAX_OUT_CHARS, "motor.left.current.limit:%d", (motion_controller.eflag.get() & COMMOTION_ERROR_M4_MAXCURRENT));
        send_diagnostics();

        // Encoders are OK if motors are off or motors are on and the encoders have registered a change within the last second.
        snprintf(buffer, MAX_OUT_CHARS, "motor.left.encoder:%d:%d",
            !(!motion_controller.is_left_on() || (motion_controller.is_left_on() && millis() - a_encoder_last_change < 1000)),
            motion_controller.a_encoder.get_latest());
        send_diagnostics();
        snprintf(buffer, MAX_OUT_CHARS, "motor.right.encoder:%d:%d",
            !(!motion_controller.is_right_on() || (motion_controller.is_right_on() && millis() - b_encoder_last_change < 1000)),
            motion_controller.b_encoder.get_latest());
        send_diagnostics();
    }

    // Odometry
    if (odometry_tracker.changed && millis() - odometry_tracker.last_report_time >= 1000) {
        odometry_tracker.changed = false;
        odometry_tracker.last_report_time = millis();

        //nh.loginfo("odometry_tracker.th:");//+String(odometry_tracker.th));
        //dtostrf(value, (decimalPlaces + 2), decimalPlaces, buf)
        //snprintf(buffer, MAX_OUT_CHARS, "Power controller state changed: %d", power_controller.power_state.get());
        //nh.loginfo(buffer);

        //strcpy(buffer, "odometry_tracker.th: ");
        //dtostrf(odometry_tracker.th, 2+6, 6, &buffer[strlen(buffer)]);
        //nh.loginfo(buffer);

        strcpy(buffer, "v_left: ");
        dtostrf(odometry_tracker.v_left, 2+6, 6, &buffer[strlen(buffer)]);
        nh.loginfo(buffer);

        strcpy(buffer, "v_right: ");
        dtostrf(odometry_tracker.v_right, 2+6, 6, &buffer[strlen(buffer)]);
        nh.loginfo(buffer);

        snprintf(buffer, MAX_OUT_CHARS, "v0:%ld:%ld:%ld:%ld",
            ftol(odometry_tracker.x), ftol(odometry_tracker.y), ftol(odometry_tracker.z), ftol(odometry_tracker.th));
        string_msg.data = buffer;
        odometry_publisher.publish(&string_msg);

        snprintf(buffer, MAX_OUT_CHARS, "v1:%ld:%ld:%ld:%ld",
            ftol(odometry_tracker.vx), ftol(odometry_tracker.vy), ftol(odometry_tracker.vz), ftol(odometry_tracker.vth));
        string_msg.data = buffer;
        odometry_publisher.publish(&string_msg);

        /*
        // CS 2017.4.9 Disabled becaues this crashes the Arduino and/or overwhelms its serial port.
        // Publish the odometry message.
        odom_msg.header.stamp = nh.now();
        odom_msg.header.frame_id = odom_link;
        odom_msg.child_frame_id = base_link;
        odom_msg.pose.pose.position.x = odometry_tracker.x;
        odom_msg.pose.pose.position.y = odometry_tracker.y;
        odom_msg.pose.pose.position.z = odometry_tracker.z;
        odom_msg.pose.pose.orientation = tf::createQuaternionFromYaw(odometry_tracker.th);
        odom_msg.twist.twist.linear.x = odometry_tracker.vx;
        odom_msg.twist.twist.linear.y = odometry_tracker.vy;
        odom_msg.twist.twist.angular.z = odometry_tracker.vth;
        odometry_publisher.publish(&odom_msg);

        // CS 2017.4.9 Disable because this immediately causes the Arduino to reset and the rosserial node to report:
        // Serial Port read failure: object of type 'int' has no len()
        // Publish the tf message.
        ts.header.stamp = nh.now();
        ts.header.frame_id = odom_link;
        ts.child_frame_id = base_link;
        ts.transform.translation.x = odometry_tracker.x;
        ts.transform.translation.y = odometry_tracker.y;
        ts.transform.translation.z = odometry_tracker.z;
        ts.transform.rotation = tf::createQuaternionFromYaw(odometry_tracker.th);
        tf_broadcaster.sendTransform(ts);
        */
    }

    // IMU sensor.
    ag_sensor.update();
    if (ag_sensor.sys_calib.get_and_clear_changed() || force_sensors) {
        int16_msg.data = ag_sensor.sys_calib.get();
        imu_calibration_sys_publisher.publish(&int16_msg);

        // http://www.cplusplus.com/reference/cstdio/snprintf/
        snprintf(buffer, MAX_OUT_CHARS, "imu_calib.sys:%d", calib_to_status[int16_msg.data]);
        send_diagnostics();

        if (int16_msg.data == 3) {
            // When the IMU is fully calibrated, save the calibration to the host.
            ag_sensor.bno.getSensorOffsets(imu_calib_struct);
            uint16ma_msg.data[0] = imu_calib_struct.accel_offset_x;
            uint16ma_msg.data[1] = imu_calib_struct.accel_offset_y;
            uint16ma_msg.data[2] = imu_calib_struct.accel_offset_z;
            uint16ma_msg.data[3] = imu_calib_struct.gyro_offset_x;
            uint16ma_msg.data[4] = imu_calib_struct.gyro_offset_y;
            uint16ma_msg.data[5] = imu_calib_struct.gyro_offset_z;
            uint16ma_msg.data[6] = imu_calib_struct.mag_offset_x;
            uint16ma_msg.data[7] = imu_calib_struct.mag_offset_y;
            uint16ma_msg.data[8] = imu_calib_struct.mag_offset_z;
            uint16ma_msg.data[9] = imu_calib_struct.accel_radius;
            uint16ma_msg.data[10] = imu_calib_struct.mag_radius;
            imu_calibration_save_publisher.publish(&uint16ma_msg);
        }
    }
    if (ag_sensor.gyr_calib.get_and_clear_changed() || force_sensors) {
        int16_msg.data = ag_sensor.gyr_calib.get();
        imu_calibration_gyr_publisher.publish(&int16_msg);

        snprintf(buffer, MAX_OUT_CHARS, "imu_calib.gyr:%d", calib_to_status[int16_msg.data]);
        send_diagnostics();
    }
    if (ag_sensor.acc_calib.get_and_clear_changed() || force_sensors) {
        int16_msg.data = ag_sensor.acc_calib.get();
        imu_calibration_acc_publisher.publish(&int16_msg);

        //This never seems to calibrate, even after following Adafruit's instructions.
        //TODO:fixme
        snprintf(buffer, MAX_OUT_CHARS, "imu_calib.acc:%d", 0);//calib_to_status[int16_msg.data]);
        send_diagnostics();
    }
    if (ag_sensor.mag_calib.get_and_clear_changed() || force_sensors) {
        int16_msg.data = ag_sensor.mag_calib.get();
        imu_calibration_mag_publisher.publish(&int16_msg);

        snprintf(buffer, MAX_OUT_CHARS, "imu_calib.mag:%d", calib_to_status[int16_msg.data]);
        send_diagnostics();
    }
    if (ag_sensor.get_and_clear_changed_euler() || ag_sensor.get_and_clear_changed_accel() || ag_sensor.get_and_clear_changed_gyro() || force_sensors) {
        // http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html

        //TODO:fix? IMU message too big, causes Arduino to crash?
//        nh.loginfo("Sending IMU packet...");
        //imu_msg.header.stamp = nh.now();
        //imu_msg.header.frame_id = base_link;
        // Our sensor returns Euler angles in degrees, but ROS requires radians.
        //imu_msg.orientation = createQuaternionFromRPY(
            //ag_sensor.ex.get_latest()*PI/180., ag_sensor.ey.get_latest()*PI/180., ag_sensor.ez.get_latest()*PI/180.);
        //imu_msg.linear_acceleration.x = ag_sensor.ax.get_latest();
        //imu_msg.linear_acceleration.y = ag_sensor.ay.get_latest();
        //imu_msg.linear_acceleration.z = ag_sensor.az.get_latest();
        //imu_publisher.publish(&imu_msg);
//        nh.loginfo("IMU packet sent.");

//        vec3_msg.x = ag_sensor.ex.get_latest()*PI/180.;
//        vec3_msg.y = ag_sensor.ey.get_latest()*PI/180.;
//        vec3_msg.z = ag_sensor.ez.get_latest()*PI/180.;
//        imu_euler_publisher.publish(&vec3_msg);
//
//        vec3_msg.x = ag_sensor.ax.get_latest();
//        vec3_msg.y = ag_sensor.ay.get_latest();
//        vec3_msg.z = ag_sensor.az.get_latest();
//        imu_accel_publisher.publish(&vec3_msg);

        // orientation(euler in radians):angular_velocity(gyro):linear_acceleration(accel in m/s^2)
        // Note, we send then as triples, because the Uno only has a 64 byte serial buffer, and sending all 9 values would take 80 bytes or more.
        snprintf(buffer, MAX_OUT_CHARS, "e:%ld:%ld:%ld",
            // orientation/euler angles in radians
            ftol(ag_sensor.ex.get_latest()*PI/180.), ftol(ag_sensor.ey.get_latest()*PI/180.), ftol(ag_sensor.ez.get_latest()*PI/180.));
        string_msg.data = buffer;
        imu_publisher.publish(&string_msg);

        snprintf(buffer, MAX_OUT_CHARS, "g:%ld:%ld:%ld",
            // angular velocity in radians/second
            ftol(ag_sensor.gx.get_latest()*PI/180.), ftol(ag_sensor.gy.get_latest()*PI/180.), ftol(ag_sensor.gz.get_latest()*PI/180.));
        string_msg.data = buffer;
        imu_publisher.publish(&string_msg);

        snprintf(buffer, MAX_OUT_CHARS, "a:%ld:%ld:%ld",
            // linear acceleration in meters/second^2
            // Note, we flip x and y to correct the axis mapping.
            ftol(ag_sensor.ay.get_latest()), ftol(ag_sensor.ax.get_latest()), ftol(ag_sensor.az.get_latest()));
        string_msg.data = buffer;
        imu_publisher.publish(&string_msg);

    }
    if (report_diagnostics) {
        snprintf(buffer, MAX_OUT_CHARS, "imu.euler.x:%d:%d", diagnostic_msgs::DiagnosticStatus::OK, static_cast<int>(ag_sensor.ex.get_latest()));
        send_diagnostics();
        snprintf(buffer, MAX_OUT_CHARS, "imu.euler.y:%d:%d", diagnostic_msgs::DiagnosticStatus::OK, static_cast<int>(ag_sensor.ey.get_latest()));
        send_diagnostics();
        snprintf(buffer, MAX_OUT_CHARS, "imu.euler.z:%d:%d", diagnostic_msgs::DiagnosticStatus::OK, static_cast<int>(ag_sensor.ez.get_latest()));
        send_diagnostics();

        // Register an error if we've fallen over.
        if (abs(ag_sensor.ex.get_latest()) >= 25 || abs(ag_sensor.ey.get_latest()) >= 25) {
            snprintf(buffer, MAX_OUT_CHARS, "imu.upright:%d", diagnostic_msgs::DiagnosticStatus::ERROR);
            send_diagnostics();
        } else if (abs(ag_sensor.ex.get_latest()) >= 10 || abs(ag_sensor.ey.get_latest()) >= 10) {
            snprintf(buffer, MAX_OUT_CHARS, "imu.upright:%d", diagnostic_msgs::DiagnosticStatus::WARN);
            send_diagnostics();
        } else {
            snprintf(buffer, MAX_OUT_CHARS, "imu.upright:%d", diagnostic_msgs::DiagnosticStatus::OK);
            send_diagnostics();
        }
    }

    // Power button.
    power_button_sensor.update();
    if (power_button_sensor.get_and_clear_changed() || force_sensors) {
        bool_msg.data = power_button_sensor.get_value();
        power_button_publisher.publish(&bool_msg);
    }
    if (report_diagnostics) {
        snprintf(buffer, MAX_OUT_CHARS, "power_button:%d:%d", diagnostic_msgs::DiagnosticStatus::OK, !power_button_sensor.get_value());
        send_diagnostics();
    }

    // Power shutoff controller.
    power_controller.update();
    if (power_controller.get_and_clear_changed()) {
        snprintf(buffer, MAX_OUT_CHARS, "Power controller state changed: %d", power_controller.power_state.get());
        nh.loginfo(buffer);
    }

    // Battery state change.
    if (battery_changed) {
        battery_changed = false;
        // http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/BatteryState.html
        battery_msg.header.stamp = nh.now();
        battery_msg.header.frame_id = base_link;
        battery_msg.voltage = battery_voltage_sensor.get_voltage();         // Voltage in Volts (Mandatory)
        //battery_msg.current = sqrt (-1); //NaN;          // Negative when discharging (A)  (If unmeasured NaN)
        //battery_msg.charge           // Current charge in Ah  (If unmeasured NaN)
        //battery_msg.capacity         // Capacity in Ah (last full capacity)  (If unmeasured NaN)
        battery_msg.design_capacity = 3; // Capacity in Ah (design capacity)  (If unmeasured NaN)
        battery_msg.percentage = battery_voltage_sensor.get_charge_ratio();       // Charge percentage on 0 to 1 range  (If unmeasured NaN)
        battery_msg.power_supply_status = get_battery_supply_status();
        battery_msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
        battery_msg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
        battery_msg.present = is_battery_present();
        //battery_msg.cell_voltage   // An array of individual cell voltages for each cell in the pack
        //battery_msg.location          // The location into which the battery is inserted. (slot number or plug)
        //battery_msg.serial_number     // The best approximation of the battery serial number
        battery_state_publisher.publish(&battery_msg);

        // Report diagnostics.
        // Note, we can't render floats or doubles with snprintf because the Arduino implementation doesn't support this.
        // https://stackoverflow.com/a/24031003/247542
        //snprintf(buffer, MAX_OUT_CHARS, "battery.voltage:%d:%f", diagnostic_msgs::DiagnosticStatus::OK, battery_msg.voltage);
        snprintf(buffer, MAX_OUT_CHARS, "battery.voltage:%d:%d", diagnostic_msgs::DiagnosticStatus::OK, static_cast<int>(battery_msg.voltage));
        send_diagnostics();

        if (battery_msg.percentage <= 0.25) {
            snprintf(buffer, MAX_OUT_CHARS, "battery.percentage:%d:%d",
                diagnostic_msgs::DiagnosticStatus::ERROR, static_cast<int>(battery_msg.percentage*100));
        } else if (battery_msg.percentage <= 0.5) {
            snprintf(buffer, MAX_OUT_CHARS, "battery.percentage:%d:%d",
                diagnostic_msgs::DiagnosticStatus::WARN, static_cast<int>(battery_msg.percentage*100));
        } else {
            snprintf(buffer, MAX_OUT_CHARS, "battery.percentage:%d:%d",
                diagnostic_msgs::DiagnosticStatus::OK, static_cast<int>(battery_msg.percentage*100));
        }
        send_diagnostics();

        if (battery_msg.present) {
            snprintf(buffer, MAX_OUT_CHARS, "battery.present:%d", diagnostic_msgs::DiagnosticStatus::OK);
        } else {
            snprintf(buffer, MAX_OUT_CHARS, "battery.present:%d", diagnostic_msgs::DiagnosticStatus::ERROR);
        }
        send_diagnostics();
    }

    force_sensors = false;

    nh.spinOnce();
    delay(1);

}
