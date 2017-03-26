/*
 * Copyright 2017 Chris Spencer
 */
// http://wiki.ros.org/rosserial_arduino/Tutorials/Hello%20World
#include <ros.h>
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
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <Wire.h>

#include "ArduinoPinout.h"
#include "I2CAddresses.h"
#include "BooleanSensor.h"
#include "BatteryVoltageSensor.h"
#include "MotionController.h"
#include "UltrasonicSensor.h"
#include "AccelGyroSensor.h"
#include "PowerController.h"

//#define PING_TIMEOUT_MS 10000

AccelGyroSensor ag_sensor = AccelGyroSensor();

MotionController motion_controller;

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
bool last_connected = false;

// http://wiki.ros.org/roscpp/Overview/Logging
// http://wiki.ros.org/rosserial/Overview/Logging
ros::NodeHandle nh;

std_msgs::Int16 int16_msg;
std_msgs::Byte byte_msg;
std_msgs::Bool bool_msg;
std_msgs::Float32 float_msg;

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

BooleanSensor external_power_sensors[2] = {
    // EP1
    // The pin measuring immediately after the external connector but before the reed switch.
    // Reads high when the external power plug is connected and external power is present.
    BooleanSensor(EXTERNAL_POWER_SENSE_1_PIN),
    // EP2
    // The pin measuring behind the reed switch.
    // Reads high when the external power plug is connected, regardless of whether or not
    // external power is present.
    // This lets the robot know if it's connected to a dead recharge station.
    BooleanSensor(EXTERNAL_POWER_SENSE_2_PIN)
    // We're only fully docked and charging when both EP1 and EP2 are high.
};

PowerController power_controller = PowerController();

BooleanSensor edge_sensors[3] = {
    BooleanSensor(EDGE_L_PIN),
    BooleanSensor(EDGE_M_PIN),
    BooleanSensor(EDGE_R_PIN)
};

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

BooleanSensor power_button_sensor = BooleanSensor(SIGNAL_BUTTON_PIN);

/*
 * End sensor definitions.
 */

/*
 * Begin publisher definitions.
 */

ros::Publisher battery_voltage_publisher = ros::Publisher("power/voltage", &float_msg);
ros::Publisher battery_charge_publisher = ros::Publisher("power/charge", &float_msg);

ros::Publisher external_power_publishers[2] = {
    ros::Publisher("power/external/0", &bool_msg),
    ros::Publisher("power/external/1", &bool_msg)
};

ros::Publisher edge_publishers[3] = {
    ros::Publisher("edge/0", &bool_msg),
    ros::Publisher("edge/1", &bool_msg),
    ros::Publisher("edge/2", &bool_msg)
};

ros::Publisher bumper_publishers[3] = {
    ros::Publisher("bumper/0", &bool_msg),
    ros::Publisher("bumper/1", &bool_msg),
    ros::Publisher("bumper/2", &bool_msg)
};

ros::Publisher ultrasonic_publishers[3] = {
    ros::Publisher("ultrasonic/0", &int16_msg),
    ros::Publisher("ultrasonic/1", &int16_msg),
    ros::Publisher("ultrasonic/2", &int16_msg)
};

ros::Publisher power_button_publisher = ros::Publisher("power/button", &bool_msg);

ros::Publisher motor_a_encoder_publisher = ros::Publisher("motor/encoder/a", &int16_msg);
ros::Publisher motor_b_encoder_publisher = ros::Publisher("motor/encoder/b", &int16_msg);
ros::Publisher motor_error_publisher = ros::Publisher("motor/error", &byte_msg);

ros::Publisher imu_calibration_sys_publisher = ros::Publisher("imu/calibration/sys", &int16_msg);
ros::Publisher imu_calibration_gyr_publisher = ros::Publisher("imu/calibration/gyr", &int16_msg);
ros::Publisher imu_calibration_acc_publisher = ros::Publisher("imu/calibration/acc", &int16_msg);
ros::Publisher imu_calibration_mag_publisher = ros::Publisher("imu/calibration/mag", &int16_msg);

/*
 * End publisher definitions.
 */

void toggle_led(){
    digitalWrite(STATUS_LED_PIN, HIGH-digitalRead(STATUS_LED_PIN));   // blink the led
}

void stop_motors(){
    nh.loginfo("Motors halted!");
    //motion_controller.stop();
    motion_controller.set(0, 0);
    //TODO:publish motor state?
}

/*
 * Begin subscribers.
 */

// rostopic pub /ultrasonics/enabled std_msgs/Bool 1
// rostopic pub /ultrasonics/enabled std_msgs/Bool 0
void on_ultrasonics_enabled(const std_msgs::Bool& msg) {
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
    motion_controller.set_cmd_vel(msg.linear.x, msg.angular.z);
}
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &on_cmd_vel);

// rostopic pub --once /torso_arduino/motor/speed std_msgs/Int16MultiArray "{layout:{dim:[], data_offset: 0}, data:[64, 64]}"
void on_motor_speed(const std_msgs::Int16MultiArray& msg){
    nh.loginfo("Setting motor speed...");
    toggle_led();
    if(motion_controller.connection_error){
        nh.loginfo("Unable to set motor speed due to connection error.");
    }else{
        //set_motor_speeds(0, 0, msg.data[0], msg.data[1]);
        motion_controller.set(msg.data[0], msg.data[1]);
        nh.loginfo("Motor speed set.");
    }
}
ros::Subscriber<std_msgs::Int16MultiArray> motor_speed_sub("motor/speed", &on_motor_speed);

//            case ID_TWIST:
//                motion_controller.set_movement(
//                    stf(packet.get_arg(0)),
//                    stf(packet.get_arg(1)),
//                    stf(packet.get_arg(2)),
//                    packet.get_arg(3).toInt()
//                );
//                ack = true;
//                break;
//                
//            case ID_MOTOR_ACCEL:
//                motion_controller.set_acceleration(packet.get_arg(0).toInt());
//                ack = true;
//                break;

//TODO:http://docs.ros.org/api/sensor_msgs/html/msg/BatteryState.html

/*
 * End subscribers.
 */

void setup() {

    // Turn on power status light.
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, true);

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

    // Register publishers.
    nh.advertise(battery_voltage_publisher);
    nh.advertise(battery_charge_publisher);
    nh.advertise(power_button_publisher);
    for (int i = 0; i < 3; i++) {
        nh.advertise(edge_publishers[i]);
    }
    for (int i = 0; i < 3; i++) {
        nh.advertise(bumper_publishers[i]);
    }
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

    // Join I2C bus as Master with address #1
    Wire.begin(1);
    Wire.setTimeout(1000L);

    // We initialize here so we can be sure it's done after the I2C initialization.
    motion_controller = MotionController();
    motion_controller.init();
    motion_controller.stop();

    ag_sensor.init();

}

void loop() {

    // If we just became disconnected from the host, then immediately halt all motors.
    if (last_connected != nh.connected() && !nh.connected()) {
        nh.loginfo("Motors stopped due to disconnect!");
        motion_controller.stop();
    }

    // Battery sensor.
    battery_voltage_sensor.update();
    if (battery_voltage_sensor.get_and_clear_changed() || force_sensors) {
        float_msg.data = battery_voltage_sensor.get_voltage();
        battery_voltage_publisher.publish(&float_msg);
    }

    // External power sensors.
    for (int i = 0; i < 2; i++) {
        external_power_sensors[i].update();
        if (external_power_sensors[i].get_and_clear_changed() || force_sensors) {
            bool_msg.data = external_power_sensors[i].value.get();
            external_power_publishers[i].publish(&bool_msg);
        }
    }

    // Edge, bumper and ultrasonic sensors.
    for (int i = 0; i < 3; i++) {
        edge_sensors[i].update();
        if (edge_sensors[i].get_and_clear_changed() || force_sensors) {
            bool_msg.data = edge_sensors[i].value.get();
            edge_publishers[i].publish(&bool_msg);
        }
        bumper_sensors[i].update();
        if (bumper_sensors[i].get_and_clear_changed() || force_sensors) {
            bool_msg.data = bumper_sensors[i].value.get();
            bumper_publishers[i].publish(&bool_msg);
        }
        if (ultrasonics_enabled) {
            ultrasonic_sensors[i].update();
            if (ultrasonic_sensors[i].get_and_clear_changed() || force_sensors) {
                int16_msg.data = ultrasonic_sensors[i].distance.get();
                ultrasonic_publishers[i].publish(&int16_msg);
            }
            delay(ultrasonics_spacing);
        }
    }

    // Motion controller.
    motion_controller.update();
    if (motion_controller.a_encoder.get_and_clear_changed() || force_sensors) {
        int16_msg.data = motion_controller.a_encoder.get_latest();
        motor_a_encoder_publisher.publish(&int16_msg);
    }
    if (motion_controller.b_encoder.get_and_clear_changed() || force_sensors) {
        int16_msg.data = motion_controller.b_encoder.get_latest();
        motor_b_encoder_publisher.publish(&int16_msg);
    }
    if (motion_controller.eflag.get_and_clear_changed() || force_sensors) {
        byte_msg.data = motion_controller.eflag.get_latest();
        motor_error_publisher.publish(&byte_msg);
    }

    // IMU sensor.
    ag_sensor.update();
    if (ag_sensor.sys_calib.get_and_clear_changed() || force_sensors) {
        int16_msg.data = ag_sensor.sys_calib.get();
        imu_calibration_sys_publisher.publish(&int16_msg);
    }
    if (ag_sensor.gyr_calib.get_and_clear_changed() || force_sensors) {
        int16_msg.data = ag_sensor.gyr_calib.get();
        imu_calibration_gyr_publisher.publish(&int16_msg);
    }
    if (ag_sensor.acc_calib.get_and_clear_changed() || force_sensors) {
        int16_msg.data = ag_sensor.acc_calib.get();
        imu_calibration_acc_publisher.publish(&int16_msg);
    }
    if (ag_sensor.mag_calib.get_and_clear_changed() || force_sensors) {
        int16_msg.data = ag_sensor.mag_calib.get();
        imu_calibration_mag_publisher.publish(&int16_msg);
    }

    // Power button shutoff controller.
    power_controller.update();

    force_sensors = false;

    nh.spinOnce();
    delay(1);

    last_connected = nh.connected();

}
