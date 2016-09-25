#include <Wire.h>

#define MASTER_ADDR 1

#define COMMOTION_ADDR1 0x1E
#define COMMOTION_ADDR2 0x1F

int read_byte(){
	while(Wire.available() < 1){}
	return Wire.read();
}

//http://arduino.stackexchange.com/a/3388/4478
uint16_t receive_wire_int(){
	
	uint16_t rxnum = 0;
	
    // Read low byte into rxnum
	rxnum += read_byte();
    
    // Read high byte into rxnum
    rxnum += read_byte() << 8;

    return rxnum;
}

void setup(){
	Wire.begin(MASTER_ADDR);        // join i2c bus (address optional for master)
	//Wire.setClock(100000L);
	Wire.setTimeout(1000L); // sets a timeout of 1000mS for I²C, should match slave
	Serial.begin(9600);  // start serial for output
}

void loop(){
	Serial.println("Requesting data..."); Serial.flush();

	// Note, you can theoretically request at most 32 bytes.
	Wire.requestFrom(COMMOTION_ADDR1, 4);
	//Wire.requestFrom(COMMOTION_ADDR2, 4);
	
	uint16_t acount = receive_wire_int();
	uint16_t bcount = receive_wire_int();

	//Serial.println(String("acount:")+String(acount));
	Serial.println(String("acount:")+String(acount)+String(" bcount:")+String(bcount));

	delay(500);
}
