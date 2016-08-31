
#include <EEPROM.h>
#include <SoftPWM.h>

#include "StringUtils.h"
#include "ArduinoPinout.h"
#include "SerialPort.h"
#include "PanController.h"
#include "TiltController.h"

#define PING_TIMEOUT_MS 10000

// Sensor declarations

PanController pan_controller = PanController(PAN_MOTOR_ENABLE, PAN_MOTOR_PHASE, PAN_MOTOR_POSITION_REF);
TiltController tilt_controller = TiltController(TILT_SERVO_POS_SET, TILT_SERVO_POS_GET);

SerialPort ser = SerialPort(115200); // must match ino.ini

//TODO:remove once Arduino IDE upgraded on RPi?
// Arduino Leonardo & Yun
#define digitalPinToInterrupt(p)  ( (p) == 0 ? 2 : ((p) == 1 ? 3 : ((p) == 2 ? 1 : ((p) == 3 ? 0 : ((p) == 7 ? 4 : -1)))) ) 

// Ping counters.
unsigned long last_ping = millis();
unsigned long total_pings = 0;
bool halted = true;

unsigned long expected_hash_sum = 1;
int led_index;
int led_state;

// If true, all sensors will push their current readings to the host, even if they haven't changed
// since last polling.
bool force_sensors = false;

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

void setup(){
    
    ser.init();
    
    attachInterrupt(digitalPinToInterrupt(PAN_MOTOR_ENCODER_A), count_pan_changes, CHANGE);
    
    //tilt_controller.enable();//TODO:remove
    
    SoftPWMBegin();
    SoftPWMSet(ULTRABRIGHT_LED_1, 0);
    SoftPWMSetFadeTime(ULTRABRIGHT_LED_1, 1000, 1000);
    SoftPWMSet(ULTRABRIGHT_LED_2, 0);
    SoftPWMSetFadeTime(ULTRABRIGHT_LED_1, 1000, 1000);
    SoftPWMSet(ULTRABRIGHT_LED_3, 0);
    SoftPWMSetFadeTime(ULTRABRIGHT_LED_1, 1000, 1000);
    
}

void halt_all_activity(){
	pan_controller.set_power(false);
    pan_controller.stop();
    tilt_controller.stop();
}

void loop(){

	//TODO:remove
	//tilt_controller.update();
//	ser.write(String(pan_controller.raw_centermark()));
//	delay(1000); return;

	// If true, response with an OK to the incoming packet.
	bool ack = false;
	
    Packet packet = ser.read();
    if(packet.is_valid() && (packet.get_id() == ID_HASH || packet.get_hash_sum() == expected_hash_sum)){
		switch(packet.get_id()){
	
			case ID_HASH:
				expected_hash_sum = packet.get_arg(0).toInt();
				break;
				
			case ID_ALL_STOP:
				// Halt all active components.
				halt_all_activity();
				ack = true;
				break;
				
			case ID_IDENTIFY:
				// Identify ourselves to the host.
				ser.write(String(ID_IDENTIFY)+String(' ')+String(HEAD));
				break;
			
			case ID_FORCE_SENSORS:
				force_sensors = true;
				ack = true;
				break;
				
			case ID_LED:
				// Set LED state.
				led_index = packet.get_arg(0).toInt(); // should be between [0-3]
				led_state = packet.get_arg(1).toInt(); // should be between [0-255]
				// LED indexes 0-2 are the separate channels for the RGB LED>
				// LED index 3 are the three ultrabright LEDs controlled in unison.
				if(led_index == 0){
					digitalWrite(STATUS_LED_RED, (bool)led_state);
				}else if(led_index == 1){
					digitalWrite(STATUS_LED_GREEN, (bool)led_state);
				}else if(led_index == 2){
					digitalWrite(STATUS_LED_BLUE, (bool)led_state);
				}else if(led_index == 3){
					SoftPWMSet(ULTRABRIGHT_LED_1, led_state);
					SoftPWMSet(ULTRABRIGHT_LED_2, led_state);
					SoftPWMSet(ULTRABRIGHT_LED_3, led_state);
				}
//				if(packet.get_data() == "0"){
//					digitalWrite(STATUS_LED_PIN, LOW);
//				}else if(packet.get_data() == "1"){
//					digitalWrite(STATUS_LED_PIN, HIGH);
//				}else if(packet.get_data() == "toggle"){
//					digitalWrite(STATUS_LED_PIN, HIGH - digitalRead(STATUS_LED_PIN));
//				}
				ack = true;
				break;
				
			case ID_LED_AUTO:
				// TODO:not implemented?
				ack = true;
				break;
				
			case ID_PING:
				// Respond to a ping request.
				last_ping = millis();
				total_pings += 1;
				halted = false;
				ser.write(String(ID_PONG)+String(' ')+String(total_pings));
				break;
				
			case ID_PAN_SPEED:
				// Get/set pan motor speed.
				if(packet.arg_length){
					pan_controller.set_speed(packet.get_data().toInt());
				}
				ser.write(String(ID_PAN_SPEED)+String(' ')+String(pan_controller.get_speed()));
				break;
				
			case ID_GO_TO_CENTER:
				if(packet.get_data() == String(NAME_PAN)){
					pan_controller.go_to_center();
				}else if(packet.get_data() == String(NAME_TILT)){
					tilt_controller.go_to_center();
				}
				ack = true;
				break;
			
			case ID_CALIBRATE:
				if(packet.get_data() == String(NAME_PAN)){
					//pan_controller.begin_calibration();
					//ser.write(String(ID_CALIBRATE)+String(' ')+String(NAME_PAN)+String(' ')+String(OK));
				}else if(packet.get_data() == String(NAME_TILT)){
					//tilt_controller.begin_calibration();
					//ser.write(String(ID_CALIBRATE)+String(' ')+String(NAME_TILT)+String(' ')+String(OK));
				}
				ack = true;
				break;
			
			case ID_PAN_ANGLE:
				pan_controller.set_target_angle(packet.get_data().toInt());
				ack = true;
				break;
			
			case ID_TILT_ANGLE:
				tilt_controller.set_target_degrees(packet.get_data().toInt(), true);
				ack = true;
				break;
			
			case ID_PAN_POWER:
				pan_controller.set_power(packet.get_data().toInt());
				ser.write(String(ID_PAN_POWER)+String(' ')+String(pan_controller.get_power()));
				break;
			
			case ID_TILT_POWER:
				tilt_controller.set_power(packet.get_data().toInt());
				ser.write(String(ID_TILT_POWER)+String(' ')+String(tilt_controller.get_power()));
				break;
				
//			case ID_GET_VALUE:
//			
//				char arg_id = packet.get_data().charAt(0);
//				switch(arg_id){
//				
//					case ID_PAN_CENTERMARK:
//						ser.write(
//							String(ID_GET_VALUE)+String(' ')+
//							String(arg_id)+String(' ')+
//							String(pan_controller.is_centermark()));
//						break;
//					
//					//DEPRECATED, remove
//					case ID_PAN_FULL_REV_COUNT:
//						ser.write(
//							String(ID_GET_VALUE)+String(' ')+
//							String(arg_id)+String(' ')+
//							String(pan_controller.count_for_full_revolution.get()));
//						break;
//					
//						
//					case ID_PAN_ANGLE:
//						ser.write(
//							String(ID_GET_VALUE)+String(' ')+
//							String(arg_id)+String(' ')+
//							String((int)pan_controller.actual_angle.get()));
//						break;
//						
//					case ID_TILT_ANGLE:
//						ser.write(
//							String(ID_GET_VALUE)+String(' ')+
//							String(arg_id)+String(' ')+
//							String((int)tilt_controller.actual_degrees.get()));
//						break;
//				
//				}
//				break;
			
			//TODO: Get/set tilt angle
		}//switch end
		if(ack){
			ser.write(String(packet.get_id())+String(' ')+String(OK));
		}
    }
    
    // Emergency shutdown if we've lost contact with host.
	if(!halted && total_pings > 10 && last_ping + PING_TIMEOUT_MS < millis()){
        halt_all_activity();
        halted = true;
    }

    // Push sensor updates to host.
    if(!halted){
        if(pan_controller.centermark.get_and_clear_changed() || force_sensors){
            ser.write(
                String(ID_GET_VALUE)+String(' ')+
                String(ID_PAN_CENTERMARK)+String(' ')+
                String(pan_controller.centermark.get()));
        }
        if(pan_controller.actual_angle.get_and_clear_changed() || force_sensors){
            ser.write(
                String(ID_GET_VALUE)+String(' ')+
                String(ID_PAN_ANGLE)+String(' ')+
                String((int)pan_controller.actual_angle.get()));
        }
        if(tilt_controller.actual_degrees.get_and_clear_changed() || force_sensors){
            ser.write(
                String(ID_GET_VALUE)+String(' ')+
                String(ID_TILT_ANGLE)+String(' ')+
                String(tilt_controller.actual_degrees.get()));
        }
//        if(pan_controller.calibrated.get_and_clear_changed() || force_sensors){
//            ser.write(
//                String(ID_GET_VALUE)+String(' ')+
//                String(ID_MOTOR_CALIBRATION)+String(' ')+
//				String(NAME_PAN)+String(' ')+
//                String((int)pan_controller.calibrated.get()));
//        }
//        if(tilt_controller.actual_degrees.get_and_clear_changed() || force_sensors){
//            ser.write(
//                String(ID_GET_VALUE)+String(' ')+
//                String(ID_MOTOR_CALIBRATION)+String(' ')+
//				String(NAME_TILT)+String(' ')+
//                String((int)tilt_controller.calibrated.get()));
//        }
    }
    force_sensors = false;
 
	// Update all sensors and controllers.
    pan_controller.update();
    tilt_controller.update();
    
}
