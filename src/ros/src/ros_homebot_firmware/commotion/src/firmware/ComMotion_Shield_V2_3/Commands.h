
#define CMD_SET_BASIC_CONFIG 1
#define CMD_SET_ENCODER_CONFIG 2
#define CMD_SET_MOTOR_CONFIG 3
#define CMD_SET_SERIAL_CONFIG 4
#define CMD_SEND_SERIAL_DATA 5
#define CMD_GET_STATUS 6
#define CMD_SET_ANGLE 7

#define CMD_RESET_EEPROM 10
#define CMD_SET_DEMO 15

#define CMD_NULL 255

#define CHASSIS_CONFIG_INDIVIDUAL 3
#define CHASSIS_CONFIG_INDIVIDUAL2 19

#define REQUESTFOR_SIZE 9

#define COMMOTION_ERROR_DISCONNECT    B01000000 // arduino can't connect via I2C
#define COMMOTION_ERROR_SHUTDOWN      B00100000 // error flag bit 6 indicates power shut down due to low battery voltage
#define COMMOTION_ERROR_LOWBAT        B00010000 // bit 5 indicates power dipping below batlow voltage
#define COMMOTION_ERROR_M4_MAXCURRENT B00001000 // bit 3 indicates M4 has exceeded current limit
#define COMMOTION_ERROR_M3_MAXCURRENT B00000100 // bit 2 indicates M3 has exceeded current limit
#define COMMOTION_ERROR_M2_MAXCURRENT B00000010 // bit 1 indicates M2 has exceeded current limit
#define COMMOTION_ERROR_M1_MAXCURRENT B00000001 // bit 0 indicates M1 has exceeded current limit
