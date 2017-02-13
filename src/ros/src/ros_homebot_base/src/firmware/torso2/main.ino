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
#include <Wire.h>

#include "ArduinoPinout.h"
#include "I2CAddresses.h"
#include "BooleanSensor.h"
#include "BatteryVoltageSensor.h"
#include "MotionController.h"
#include "UltrasonicSensor.h"
#include "AccelGyroSensor.h"

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

ros::NodeHandle nh;

std_msgs::Int16 int16_msg;
std_msgs::Byte byte_msg;
std_msgs::Bool bool_msg;
std_msgs::Float32 float_msg;

// 252 bytes, 75% => 88%
//float vec2[2];
//std_msgs::Float32MultiArray vec2_msg;

//float float_vec3[3];
//std_msgs::Float32MultiArray float_vec3_msg; // 252 bytes

//int int_vec3[3];
//std_msgs::Int8MultiArray int_vec3_msg;

//std_msgs::Float32 float_msg; // 68 bytes

//float vec3[3];
//std_msgs::Float64MultiArray vec3_msg;
//vec3_msg.data_length = 3;

//String str_buffer;
//std_msgs::String str_msg; // 64 bytes

// Each publisher instance consumes 18 bytes of SRAM, so let's not make too many of these.
// This will report the edge, bumper and power button boolean sensors.
//ros::Publisher boolean_sensor_publisher = ros::Publisher("booleans", &byte_msg);

//ros::Publisher battery_voltage_publisher("battery/voltage", &float_msg);

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
    BooleanSensor(EXTERNAL_POWER_SENSE_1_PIN),
    BooleanSensor(EXTERNAL_POWER_SENSE_2_PIN)
};

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

//ros::Publisher imu_calibration_publisher("imu/calibration", &byte_msg);

/*
 * End publisher definitions.
 */

/*
 * Begin subscribers.
 */

void on_ultrasonics_enabled(const std_msgs::Bool& msg) {
    ultrasonics_enabled = msg.data;
}
ros::Subscriber<std_msgs::Bool> ultrasonics_enabled_sub("ultrasonics/enabled", &on_ultrasonics_enabled);

void on_halt(const std_msgs::Empty& msg) {
    motion_controller.stop();
}
ros::Subscriber<std_msgs::Empty> halt_sub("halt", &on_halt);

void on_force_sensors(const std_msgs::Empty& msg) {
    force_sensors = true;
}
ros::Subscriber<std_msgs::Empty> force_sensors_sub("force_sensors", &on_force_sensors);

void on_toggle_led(const std_msgs::Empty& msg) {
    digitalWrite(STATUS_LED_PIN, HIGH-digitalRead(STATUS_LED_PIN));   // blink the led
}
ros::Subscriber<std_msgs::Empty> toggle_led_sub("toggle_led", &on_toggle_led);

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

    // Join I2C bus as Master with address #1
    Wire.begin(1);
    Wire.setTimeout(1000L);

    // We initialize here so we can be sure it's done after the I2C initialization.
    motion_controller = MotionController();
    motion_controller.init();
    motion_controller.stop();

    //ag_sensor.init();

}

void loop() {

    battery_voltage_sensor.update();
    if (battery_voltage_sensor.get_and_clear_changed() || force_sensors) {
        float_msg.data = battery_voltage_sensor.get_voltage();
        battery_voltage_publisher.publish(&float_msg);
    }

    for (int i = 0; i < 2; i++) {
        external_power_sensors[i].update();
        if (external_power_sensors[i].get_and_clear_changed() || force_sensors) {
            bool_msg.data = external_power_sensors[i].value.get();
            external_power_publishers[i].publish(&bool_msg);
        }
    }

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

    if (motion_controller.get_and_clear_changed() || force_sensors) {
        int16_msg.data = motion_controller.a_encoder.get();
        motor_a_encoder_publisher.publish(&int16_msg);

        int16_msg.data = motion_controller.b_encoder.get();
        motor_b_encoder_publisher.publish(&int16_msg);

        int16_msg.data = motion_controller.eflag.get();
        motor_error_publisher.publish(&int16_msg);
    }

//    if (ag_sensor.get_and_clear_changed_calibration() || force_sensors) {
//        //TODO:these cause the serial port to crash?
////        if(ag_sensor.is_gyr_calibrated() && ag_sensor.get_and_clear_changed_euler()){
////            //ser.write(ag_sensor.get_reading_packet_euler(force_sensors));
////            float64_array_msg.data.resize(3);
////            float64_array_msg.data[0] = ag_sensor.ex.get_latest();
////            float64_array_msg.data[1] = ag_sensor.ey.get_latest();
////            float64_array_msg.data[2] = ag_sensor.ez.get_latest();
////        }
////        if(ag_sensor.is_mag_calibrated())
////        ser.write(ag_sensor.get_reading_packet_magnetometer(force_sensors));
////        ser.write(ag_sensor.get_reading_packet_calibration(force_sensors));
//    }

    motion_controller.update();
    //power_controller.update();

    force_sensors = false;

    nh.spinOnce();
    delay(1);
}
