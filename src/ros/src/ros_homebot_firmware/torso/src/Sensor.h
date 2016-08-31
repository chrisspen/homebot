
#ifndef Sensor_h
#define Sensor_h

#include "SerialPort.h"

// Abstract
class Sensor{

public:

    virtual void update();
    //{
    	// called to refresh sensor input for pull sensor types
    	// OVERRIDE
    //}

    virtual bool get_and_clear_changed();
    //{
    	// Returns true if something has changed since the last call, and if so, resets the flag.
    	// Returns false if nothing has changed.
    	//return false; // OVERRIDE
    //}

    virtual String get_reading_packet();
    //{
    	// Returns a packet string of data to send to the host.
        //return String(); // OVERRIDE
    //}

    void send_pending(SerialPort ser, bool force){
    	// Sends packets to the host via the given serial connection.
        if(force || get_and_clear_changed()){
            ser.write(get_reading_packet());
        }
    }

};

#endif
