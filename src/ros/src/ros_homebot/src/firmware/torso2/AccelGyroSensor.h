
#include <Wire.h>
#include "Adafruit_BNO055/Adafruit_Sensor.h"
#include "Adafruit_BNO055/Adafruit_BNO055.h"
#include "Adafruit_BNO055/utility/imumaths.h"

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
#define BNO055_SAMPLERATE_DELAY_MS 100
//#define BNO055_SAMPLERATE_DELAY_MS 1000

//#define BNO055_SAMPLERATE_TIMEOUT_MS 60000 // 1 minute #TODO:revert
#define BNO055_SAMPLERATE_TIMEOUT_MS 10000 // 10 seconds

#define BNO055_UNCALIBRATED 0
#define BNO055_CALIBRATION_STARTED 1
#define BNO055_CALIBRATION_COMPLETING 2
#define BNO055_CALIBRATED 3

class AccelGyroSensor: public Sensor{

    private:

    public:

        Adafruit_BNO055 bno;

        // Possible vector values can be:
        // - VECTOR_ACCELEROMETER - m/s^2
        // - VECTOR_MAGNETOMETER  - uT
        // - VECTOR_GYROSCOPE     - rad/s
        // - VECTOR_EULER         - degrees
        // - VECTOR_LINEARACCEL   - m/s^2
        // - VECTOR_GRAVITY       - m/s^2
    
        ChangeTracker<bool> connected = ChangeTracker<bool>(false);

        ChangeTracker<double> ax = ChangeTracker<double>(0, 0, BNO055_SAMPLERATE_DELAY_MS);
        ChangeTracker<double> ay = ChangeTracker<double>(0, 0, BNO055_SAMPLERATE_DELAY_MS);
        ChangeTracker<double> az = ChangeTracker<double>(0, 0, BNO055_SAMPLERATE_DELAY_MS);

        ChangeTracker<double> ex = ChangeTracker<double>(0, 0, BNO055_SAMPLERATE_DELAY_MS);
        ChangeTracker<double> ey = ChangeTracker<double>(0, 0, BNO055_SAMPLERATE_DELAY_MS);
        ChangeTracker<double> ez = ChangeTracker<double>(0, 0, BNO055_SAMPLERATE_DELAY_MS);

        ChangeTracker<double> gx = ChangeTracker<double>(0, 0, BNO055_SAMPLERATE_DELAY_MS);
        ChangeTracker<double> gy = ChangeTracker<double>(0, 0, BNO055_SAMPLERATE_DELAY_MS);
        ChangeTracker<double> gz = ChangeTracker<double>(0, 0, BNO055_SAMPLERATE_DELAY_MS);

        ChangeTracker<double> mx = ChangeTracker<double>(0, 0, BNO055_SAMPLERATE_DELAY_MS);
        ChangeTracker<double> my = ChangeTracker<double>(0, 0, BNO055_SAMPLERATE_DELAY_MS);
        ChangeTracker<double> mz = ChangeTracker<double>(0, 0, BNO055_SAMPLERATE_DELAY_MS);

        // set to 0 if not calibrated and 3 if fully calibrated.
        ChangeTracker<uint8_t> sys_calib = ChangeTracker<uint8_t>(0, 0, BNO055_SAMPLERATE_DELAY_MS, BNO055_SAMPLERATE_TIMEOUT_MS);
        ChangeTracker<uint8_t> gyr_calib = ChangeTracker<uint8_t>(0, 0, BNO055_SAMPLERATE_DELAY_MS, BNO055_SAMPLERATE_TIMEOUT_MS);
        ChangeTracker<uint8_t> acc_calib = ChangeTracker<uint8_t>(0, 0, BNO055_SAMPLERATE_DELAY_MS, BNO055_SAMPLERATE_TIMEOUT_MS);
        ChangeTracker<uint8_t> mag_calib = ChangeTracker<uint8_t>(0, 0, BNO055_SAMPLERATE_DELAY_MS, BNO055_SAMPLERATE_TIMEOUT_MS);

        AccelGyroSensor(){
            // Note default address is BNO055_ADDRESS_A=0x28
            bno = Adafruit_BNO055();
        }
        
        void init(){
            connected.set(bno.begin());
            if(connected.get_latest()){
                bno.setExtCrystalUse(true);
                
                // Change x to z.
                // Has no effect.
                //bno.write8(Adafruit_BNO055::adafruit_bno055_reg_t::BNO055_AXIS_MAP_CONFIG_ADDR, 0x9);
                //delay(10);
            }
        }

        double get_absolute_tilt_x(){
            return abs(ex.get_latest());//logical z?
        }

        double get_absolute_tilt_y(){
            return abs(ey.get_latest());//logical x?
        }

        double get_absolute_tilt_z(){
            return abs(ez.get_latest());//logical y?
        }

        virtual void update(){
            imu::Vector<3> vector;

            if(!connected.get_latest()) return false;

            uint8_t _sys, _gyr, _acc, _mag = 0;
            bno.getCalibration(&_sys, &_gyr, &_acc, &_mag);
            sys_calib.set(_sys);
            gyr_calib.set(_gyr);
            acc_calib.set(_acc);
            mag_calib.set(_mag);

            // Absolute orientation/Euler angles, in degrees.
            vector = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
            //ex.set(vector.x());
            //ey.set(vector.y());
            //ez.set(vector.z());
            // Remap axis so z points "up", x points "forward", and y points "sideways"
            ex.set(vector.y());
            ey.set(vector.z());
            ez.set(vector.x());

            // Linear acceleration, in meters/second^2.
            vector = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
            ax.set(vector.x());
            ay.set(vector.y());
            az.set(vector.z());
            
            // Angular velocity, in degrees/second.
            // Note, datasheet page 30 allows for selection of units
            vector = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
            gx.set(vector.x());
            gy.set(vector.y());
            gz.set(vector.z());

        }
        
        virtual bool get_and_clear_changed(){
            return true;
        }
        
        bool get_and_clear_changed_euler(){
            return ex.get_and_clear_changed() || ey.get_and_clear_changed() || ez.get_and_clear_changed();
        }

        bool get_and_clear_changed_accel(){
            return ax.get_and_clear_changed() || ay.get_and_clear_changed() || az.get_and_clear_changed();
        }

        bool get_and_clear_changed_gyro(){
            return gx.get_and_clear_changed() || gy.get_and_clear_changed() || gz.get_and_clear_changed();
        }

//        String get_reading_packet_accelerometer(bool force){
//          if(!connected.get_latest()) return String("");
//          imu::Vector<3> vector = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
//          ax.set(vector.x());
//          ay.set(vector.y());
//          az.set(vector.z());
//          if(force || ax.get_and_clear_changed() || ay.get_and_clear_changed() || az.get_and_clear_changed()){
//              return String(ID_GET_VALUE)+String(' ')+
//                  String(ID_IMU_ACCELEROMETER)+String(' ')+
//                  String(fts(vector.x()))+String(' ')+
//                  String(fts(vector.y()))+String(' ')+
//                  String(fts(vector.z()));
//          }else{
//              return String("");
//          }
//      }

//        String get_reading_packet_euler(bool force){
//          if(!connected.get_latest()) return String("");
//          imu::Vector<3> vector = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
//          ex.set(vector.x());
//          ey.set(vector.y());
//          ez.set(vector.z());
//          if(force || ex.get_and_clear_changed() || ey.get_and_clear_changed() || ez.get_and_clear_changed()){
//              return String(ID_GET_VALUE)+String(' ')+
//                  String(ID_IMU_EULER)+String(' ')+
//                  String(fts(vector.x()))+String(' ')+
//                  String(fts(vector.y()))+String(' ')+
//                  String(fts(vector.z()));
//          }else{
//              return String("");
//          }
//      }

//        String get_reading_packet_gyroscope(bool force){
//          if(!connected.get_latest()) return String("");
//          imu::Vector<3> vector = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
//          gx.set(vector.x());
//          gy.set(vector.y());
//          gz.set(vector.z());
//          if(force || gx.get_and_clear_changed() || gy.get_and_clear_changed() || gz.get_and_clear_changed()){
//              return String(ID_GET_VALUE)+String(' ')+
//                  String(ID_IMU_GYROSCOPE)+String(' ')+
//                  String(fts(vector.x()))+String(' ')+
//                  String(fts(vector.y()))+String(' ')+
//                  String(fts(vector.z()));
//          }else{
//              return String("");
//          }
//      }

//        String get_reading_packet_magnetometer(bool force){
//          if(!connected.get_latest()) return String("");
//          imu::Vector<3> vector = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
//          mx.set(vector.x());
//          my.set(vector.y());
//          mz.set(vector.z());
//          if(force || mx.get_and_clear_changed() || my.get_and_clear_changed() || mz.get_and_clear_changed()){
//              return String(ID_GET_VALUE)+String(' ')+
//                  String(ID_IMU_MAGNETOMETER)+String(' ')+
//                  String(fts(vector.x()))+String(' ')+
//                  String(fts(vector.y()))+String(' ')+
//                  String(fts(vector.z()));
//          }else{
//              return String("");
//          }
//      }

        bool is_sys_calibrated(){
            return sys_calib.get_latest();
        }

        bool is_acc_calibrated(){
            return acc_calib.get_latest();
        }

        bool is_mag_calibrated(){
            return mag_calib.get_latest();
        }

        bool is_gyr_calibrated(){
            return gyr_calib.get_latest();
        }


//        bool get_and_clear_changed_calibration(){
//            if(!connected.get_latest()) return false;
//            uint8_t _sys, _gyr, _acc, _mag = 0;
//            bno.getCalibration(&_sys, &_gyr, &_acc, &_mag);
//            sys_calib.set(_sys);
//            gyr_calib.set(_gyr);
//            acc_calib.set(_acc);
//            mag_calib.set(_mag);
//            return connected.get_and_clear_changed() || sys_calib.get_and_clear_changed() || gyr_calib.get_and_clear_changed() || acc_calib.get_and_clear_changed() || mag_calib.get_and_clear_changed();
//        }

//        String get_reading_packet_calibration(bool force){
//          if(!connected.get_latest()) return String("");
//          uint8_t _sys, _gyr, _acc, _mag = 0;
//          bno.getCalibration(&_sys, &_gyr, &_acc, &_mag);
//          sys_calib.set(_sys);
//          gyr_calib.set(_gyr);
//          acc_calib.set(_acc);
//          mag_calib.set(_mag);
//          if(force || sys_calib.get_and_clear_changed() || gyr_calib.get_and_clear_changed() || acc_calib.get_and_clear_changed() || mag_calib.get_and_clear_changed()){
//              return String(ID_GET_VALUE)+String(' ')+
//                  String(ID_IMU_CALIBRATION)+String(' ')+
//                  String(_sys)+String(' ')+
//                  String(_gyr)+String(' ')+
//                  String(_acc)+String(' ')+
//                  String(_mag);
//          }else{
//              return String("");
//          }
//      }

};
