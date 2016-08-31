
#include "Wire.h"

#include "ArduinoPinout.h"
#include "I2CAddresses.h"
#include "SerialPort.h"

#include "ArduinoTemperatureSensor.h"
#include "BatteryVoltageSensor.h"
#include "BumperSensor.h"
#include "EdgeSensor.h"
#include "ExternalPowerSensor.h"
#include "MotionController.h"
#include "PowerController.h"
#include "StatusLightController.h"
#include "UltrasonicSensor.h"
//#include "BatteryTemperatureSensor.h"
#include "AccelGyroSensor.h"

#define PING_TIMEOUT_MS 10000

// Sensor declarations

BumperSensor bumper[3] = {
    BumperSensor(BUMPER_L_PIN, 1),
	BumperSensor(BUMPER_M_PIN, 2),
	BumperSensor(BUMPER_R_PIN, 3)
};

EdgeSensor edge[3] = {
	EdgeSensor(EDGE_L_PIN, 1),
	EdgeSensor(EDGE_M_PIN, 2),
	EdgeSensor(EDGE_R_PIN, 3)
};

UltrasonicSensor sonic[3] = {
	UltrasonicSensor(SONIC_L_PIN, 1),
	UltrasonicSensor(SONIC_M_PIN, 2),
	UltrasonicSensor(SONIC_R_PIN, 3)
};

ArduinoTemperatureSensor arduino_temperature_sensor = ArduinoTemperatureSensor();

AccelGyroSensor ag_sensor = AccelGyroSensor();

// Controller declarations.

MotionController motion_controller;

SerialPort ser = SerialPort(38400); // must match ano.ini TODO

PowerController power_controller = PowerController(POWER_OFF_PIN, ser);

// Space each reading to prevent echo from interfering
// with sequential readings.
int sonar_spacing = 10;

bool sonar_enabled = false;

unsigned long last_sonar = millis();

// Ping counters.
unsigned long last_ping = millis();
unsigned long total_pings = 0;
bool halted = true;

//Sensor *sensors[255];

void setup(){
	
    ser.init();

    // Join I2C bus as Master with address #1
    Wire.begin(1);

    // We initialize here so we can be sure it's done after the I2C initialization.
    motion_controller = MotionController();
    motion_controller.init();
    
    // The ComMotion motor controller operates independently and will continue performing
    // the last action before it was shutdown. In case we were moving when we last shutdown,
    // ensure the controller is now stopped.
    halt_all_activity();

    //ser.log(F("Initializing accel/gyro..."));
    ag_sensor.init();
    
    //sensors[ID_LED] = (Sensor *)&power_controller;
    //x = classes[i]->myFunction(z);
    
}

unsigned long expected_hash_sum = 1;

// If true, all sensors will push their current readings to the host, even if they haven't changed
// since last polling.
bool force_sensors = false;

void halt_all_activity(){
	motion_controller.stop();
}

void loop(){
	
	// If true, response with an OK to the incoming packet.
	bool ack = false;
	
    /*
    //TODO:remove
    Serial.println(String(millis()));
    Serial.flush();
    delay(1000);
    */

	/*
	//TODO:remove
	// Test bad input on D13, status button not working.
	power_controller.status_button.update();
    Serial.println(String("is_pressed.raw:")+String(digitalRead(SIGNAL_BUTTON_PIN)));
    Serial.println(String("is_pressed.sensor:")+String(power_controller.status_button.is_pressed()));
    Serial.flush();
    delay(1000);
    */
	
    Packet packet = ser.read();
    if(packet.is_valid() && (packet.get_id() == ID_HASH || packet.get_hash_sum() == expected_hash_sum)){
		switch(packet.get_id()){
		
			case ID_HASH:
				expected_hash_sum = packet.get_arg(0).toInt();
				break;
				
			case ID_IDENTIFY:
				// Identify ourselves to the host.
				ser.write(String(ID_IDENTIFY)+String(' ')+String(TORSO));
				break;
			
			case ID_FORCE_SENSORS:
				force_sensors = true;
				ack = true;
				break;
		
			case ID_ALL_STOP:
				// Halt all active components.
				halt_all_activity();
				ack = true;
				break;
				
			case ID_LED:
				// Set LED state.
				// The packet contains (index,state) but we only have one LED so we ignore the index.
				power_controller.status_light.set((bool)(packet.get_arg(1).toInt()));
				ack = true;
				break;
				
			case ID_LED_AUTO:
				power_controller.status_light_auto = packet.get_boolean();
				ack = true;
				break;
			
			case ID_SONAR_POWER:
				sonar_enabled = packet.get_boolean();
				if(!sonar_enabled){
					sonic[0].duration.set(0);
					sonic[1].duration.set(0);
					sonic[2].duration.set(0);
				}
				ack = true;
				break;
				
			case ID_PING:
				// Respond to a ping request.
				last_ping = millis();
				total_pings += 1;
				halted = false;
				ser.write(String(ID_PONG)+String(' ')+String(total_pings));
				break;
				
			case ID_MOTOR_SPEED:
				if(packet.arg_length >= 2){
					int left_speed = packet.get_arg(0).toInt();
					int right_speed = packet.get_arg(1).toInt();
					motion_controller.set(left_speed, right_speed);
				}
				ack = true;
				break;
				
			case ID_MOTOR_ACCEL:
				motion_controller.set_acceleration(packet.get_arg(0).toInt());
				ack = true;
				break;
				
			case ID_SHUTDOWN:
				power_controller.shutdown();
				ack = true;
				break;
		
//			case ID_GET_VALUE:
//			
//				char arg_id = packet.get_data().charAt(0);
//				switch(arg_id){
//					
//					case ID_LED:
//						ser.write(
//							String(ID_GET_VALUE)+String(' ')+
//							String(arg_id)+String(' ')+
//							String(digitalRead(STATUS_LED_PIN)));
//						break;
//						
//					case ID_LED_AUTO:
//						ser.write(
//							String(ID_GET_VALUE)+String(' ')+
//							String(arg_id)+String(' ')+
//							String(power_controller.status_light_auto));
//						break;
//					
//					case ID_BATTERY_VOLTAGE:
//						ser.write(battery_voltage_sensor.get_reading_packet());
//						break;
//					
//					case ID_BATTERY_CHARGE_RATIO:
//						ser.write(String(battery_voltage_sensor.get_reading_packet2()));
//						break;
//					
//					//case ID_BATTERY_TEMP:
//					//	ser.write(String(battery_temperature_sensor.get_reading_packet()));
//					//	break;
//					
//					case ID_BUMPER:
//						
//						bumper_index = String(packet.get_data().charAt(1)).toInt();
//						ser.write(bumper[bumper_index-1].get_reading_packet());
//						break;
//						
//					case ID_ULTRASONIC:
//					
//						bumper_index = String(packet.get_data().charAt(1)).toInt();
//						ser.write(sonic[bumper_index-1].get_reading_packet());
//						break;
//						
//					case ID_EDGE:
//					
//						bumper_index = String(packet.get_data().charAt(1)).toInt();
//						ser.write(edge[bumper_index-1].get_reading_packet());
//						break;
//			 
////					case ID_ARDUINO_TEMP:
////						ser.write(arduino_temperature_sensor.get_reading_packet());
////						break;
//					
//					case ID_EXTERNAL_POWER:
//						ser.write(external_power_sensor.get_reading_packet());
//						break;
//						
//					case ID_ACCELGYRO:
//						ser.write(ag_sensor.get_reading_packet());
//						break;
//						
//					case ID_MOTOR_ACCEL:
//						ser.write(motion_controller.get_acceleration_packet());
//						break;
//						
//				}
//				break;
			
			//TODO: Get/set motor speed
	
		}//switch end
		if(ack){
			ser.write(String(packet.get_id())+String(' ')+String(OK));
		}
    }
    
    // Push sensor updates to host.
    if(!halted){
    	for(int i=0; i<3; i++){
            bumper[i].send_pending(ser, force_sensors);
            edge[i].send_pending(ser, force_sensors);
            if(sonar_enabled){
    			sonic[i].send_pending(ser, force_sensors);
    		}
    	}
    	
    	arduino_temperature_sensor.send_pending(ser, force_sensors);
    	power_controller.status_button.send_pending(ser, force_sensors);
    	power_controller.battery_voltage_sensor.send_pending(ser, force_sensors);
    	power_controller.external_power_sensor.send_pending(ser, force_sensors);
    	
    	ser.write(ag_sensor.get_reading_packet_accelerometer(force_sensors));
    	ser.write(ag_sensor.get_reading_packet_euler(force_sensors));
    	ser.write(ag_sensor.get_reading_packet_magnetometer(force_sensors));
    	ser.write(ag_sensor.get_reading_packet_calibration(force_sensors));
    }
    force_sensors = false;
    
    // Emergency shutdown if we've lost contact with host.
    if(!halted && total_pings > 10 && last_ping + PING_TIMEOUT_MS < millis()){
        halt_all_activity();
        halted = true;
    }
	
    // Update all sensors and controllers.
    if(sonar_enabled && last_sonar + 50 < millis()){
    	for(int i=0; i<3; i++){
        	sonic[i].update();
            delay(sonar_spacing);
        }
    	last_sonar = millis();
    }
  	for(int i=0; i<3; i++){
    	bumper[i].update();
        edge[i].update();
    }
    arduino_temperature_sensor.update();
    power_controller.update();
    motion_controller.update();
	
}
