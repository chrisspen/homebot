
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "ID.h"
#include "ChangeTracker.h"
#include "Sensor.h"

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

class AccelGyroSensor: public Sensor{

    private:

        Adafruit_BNO055 bno;

    public:
    
        bool connected = false;

		// Possible vector values can be:
		// - VECTOR_ACCELEROMETER - m/s^2
		// - VECTOR_MAGNETOMETER  - uT
		// - VECTOR_GYROSCOPE     - rad/s
		// - VECTOR_EULER         - degrees
		// - VECTOR_LINEARACCEL   - m/s^2
		// - VECTOR_GRAVITY       - m/s^2
    
        ChangeTracker<double> ax = ChangeTracker<double>(0, 0, 100);
        ChangeTracker<double> ay = ChangeTracker<double>(0, 0, 100);
        ChangeTracker<double> az = ChangeTracker<double>(0, 0, 100);

        ChangeTracker<double> ex = ChangeTracker<double>(0, 0, 100);
        ChangeTracker<double> ey = ChangeTracker<double>(0, 0, 100);
        ChangeTracker<double> ez = ChangeTracker<double>(0, 0, 100);

        ChangeTracker<double> gx = ChangeTracker<double>(0, 0, 100);
        ChangeTracker<double> gy = ChangeTracker<double>(0, 0, 100);
        ChangeTracker<double> gz = ChangeTracker<double>(0, 0, 100);

        ChangeTracker<double> mx = ChangeTracker<double>(0, 0, 100);
        ChangeTracker<double> my = ChangeTracker<double>(0, 0, 100);
        ChangeTracker<double> mz = ChangeTracker<double>(0, 0, 100);

        ChangeTracker<uint8_t> sys_calib = ChangeTracker<uint8_t>(0, 0, 1000);
        ChangeTracker<uint8_t> gyr_calib = ChangeTracker<uint8_t>(0, 0, 1000);
        ChangeTracker<uint8_t> acc_calib = ChangeTracker<uint8_t>(0, 0, 1000);
        ChangeTracker<uint8_t> mag_calib = ChangeTracker<uint8_t>(0, 0, 1000);

        AccelGyroSensor(){
        	// Note default address is BNO055_ADDRESS_A=0x28
            bno = Adafruit_BNO055();
        }
        
        void init(){
        	connected = bno.begin();
			bno.setExtCrystalUse(true);
        }

        virtual void update(){
        }
        
        virtual bool get_and_clear_changed(){
        	return true;
        }

        virtual String get_reading_packet(){
        	return String("");
        }
        
        String get_reading_packet_accelerometer(bool force){
			imu::Vector<3> vector = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
			ax.set(vector.x());
			ay.set(vector.y());
			az.set(vector.z());
			if(force || ax.get_and_clear_changed() || ay.get_and_clear_changed() || az.get_and_clear_changed()){
				return String(ID_GET_VALUE)+String(' ')+
					String(ID_IMU_ACCELEROMETER)+String(' ')+
					String(fts(vector.x()))+String(' ')+
					String(fts(vector.y()))+String(' ')+
					String(fts(vector.z()));
			}else{
				return String("");
			}
		}

        String get_reading_packet_euler(bool force){
			imu::Vector<3> vector = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
			ex.set(vector.x());
			ey.set(vector.y());
			ez.set(vector.z());
			if(force || ex.get_and_clear_changed() || ey.get_and_clear_changed() || ez.get_and_clear_changed()){
				return String(ID_GET_VALUE)+String(' ')+
					String(ID_IMU_EULER)+String(' ')+
					String(fts(vector.x()))+String(' ')+
					String(fts(vector.y()))+String(' ')+
					String(fts(vector.z()));
			}else{
				return String("");
			}
		}

        String get_reading_packet_gyroscope(bool force){
			imu::Vector<3> vector = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
			gx.set(vector.x());
			gy.set(vector.y());
			gz.set(vector.z());
			if(force || gx.get_and_clear_changed() || gy.get_and_clear_changed() || gz.get_and_clear_changed()){
				return String(ID_GET_VALUE)+String(' ')+
					String(ID_IMU_GYROSCOPE)+String(' ')+
					String(fts(vector.x()))+String(' ')+
					String(fts(vector.y()))+String(' ')+
					String(fts(vector.z()));
			}else{
				return String("");
			}
		}

        String get_reading_packet_magnetometer(bool force){
			imu::Vector<3> vector = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
			mx.set(vector.x());
			my.set(vector.y());
			mz.set(vector.z());
			if(force || mx.get_and_clear_changed() || my.get_and_clear_changed() || mz.get_and_clear_changed()){
				return String(ID_GET_VALUE)+String(' ')+
					String(ID_IMU_MAGNETOMETER)+String(' ')+
					String(fts(vector.x()))+String(' ')+
					String(fts(vector.y()))+String(' ')+
					String(fts(vector.z()));
			}else{
				return String("");
			}
		}

        String get_reading_packet_calibration(bool force){
			uint8_t _sys, _gyr, _acc, _mag = 0;
			bno.getCalibration(&_sys, &_gyr, &_acc, &_mag);
			sys_calib.set(_sys);
			gyr_calib.set(_gyr);
			acc_calib.set(_acc);
			mag_calib.set(_mag);
			if(force || sys_calib.get_and_clear_changed() || gyr_calib.get_and_clear_changed() || acc_calib.get_and_clear_changed() || mag_calib.get_and_clear_changed()){
				return String(ID_GET_VALUE)+String(' ')+
					String(ID_IMU_CALIBRATION)+String(' ')+
					String(_sys)+String(' ')+
					String(_gyr)+String(' ')+
					String(_acc)+String(' ')+
					String(_mag);
			}else{
				return String("");
			}
		}

};
