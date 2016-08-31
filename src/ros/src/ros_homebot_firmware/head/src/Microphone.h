
#ifndef Microphone_h
#define Microphone_h

#include "ChangeTracker.h"

class Microphone{

	private:

		// Pin the microphone is attached to.
		int _pin;

		// The rate at which sound is measured on the pin.
		// bits/sec
		// 1 measurement=8 bits
		unsigned long _bitrate;

		// The time the pin was last measured.
		unsigned long _last_update;

	public:

		bool enabled = false;

		ChangeTracker<int> value = ChangeTracker<int>(0);

		Microphone(int pin, unsigned long bitrate=16000){
			_pin = pin;
			_bitrate = bitrate;
			_last_update = millis();
			pinMode(_pin, INPUT);
		}

		void update(){
			if(!enabled || millis() - _last_update > (8./_bitrate*1000)){
				return;
			}
			value.set(analogRead(_pin));
			_last_update = millis();
		}

};

#endif
