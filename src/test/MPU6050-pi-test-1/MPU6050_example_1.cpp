/*
I2Cdev library collection - MPU6050 RPi example
Based on the example in Arduino/MPU6050/

==============================================
I2Cdev device library code is placed under the MIT license

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================

To compile on a Raspberry Pi (1 or 2)
  1. install the bcm2835 library, see http://www.airspayce.com/mikem/bcm2835/index.html
  2. enable i2c on your RPi , see https://learn.adafruit.com/adafruits-raspberry-pi-lesson-4-gpio-setup/configuring-i2c
  3. connect your i2c devices
  4. then from bash
      $ PATH_I2CDEVLIB=~/i2cdevlib/
      $ gcc -o MPU6050_example_1 ./MPU6050_example_1.cpp \
         -I ${PATH_I2CDEVLIB}RaspberryPi_bcm2835/I2Cdev ${PATH_I2CDEVLIB}RaspberryPi_bcm2835/I2Cdev/I2Cdev.cpp \
         -I ${PATH_I2CDEVLIB}/Arduino/MPU6050/ ${PATH_I2CDEVLIB}/Arduino/MPU6050/MPU6050.cpp -l bcm2835 -l m
      $ sudo ./MPU6050_example_1

*/

#include <stdio.h>
#include <bcm2835.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <math.h>

#include "AccelGyroSensorOffsets.h"

int main(int argc, char **argv) {
  printf("MPU6050 3-axis acceleromter example program\n");
  I2Cdev::initialize();
  MPU6050 mpu ;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  if ( mpu.testConnection() )
    printf("MPU6050 connection test successful\n") ;
  else {
    fprintf( stderr, "MPU6050 connection test failed! something maybe wrong, continuing anyway though ...\n");
    //return 1;
  }
  mpu.initialize();
  // use the code below to change accel/gyro offset values
  /*
  printf("Updating internal sensor offsets...\n");
  // -76	-2359	1688	0	0	0
  printf("%i \t %i \t %i \t %i \t %i \t %i\n",
	 mpu.getXAccelOffset(),
	 mpu.getYAccelOffset(),
	 mpu.getZAccelOffset(),
	 mpu.getXGyroOffset(),
	 mpu.getYGyroOffset(),
	 mpu.getZGyroOffset());
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  printf("%i \t %i \t %i \t %i \t %i \t %i\n",
	 mpu.getXAccelOffset(),
	 mpu.getYAccelOffset(),
	 mpu.getZAccelOffset(),
	 mpu.getXGyroOffset(),
	 mpu.getYGyroOffset(),
	 mpu.getZGyroOffset());

// Ordered as ax,ay,az,gx,gy,gz.
int mpu6050_offsets[6] = {-3984,-525,-849,17,147,54};
  */

  mpu.setXAccelOffset(mpu6050_offsets[0]);
  mpu.setYAccelOffset(mpu6050_offsets[1]);
  mpu.setZAccelOffset(mpu6050_offsets[2]);
  mpu.setXGyroOffset(mpu6050_offsets[3]);
  mpu.setYGyroOffset(mpu6050_offsets[4]);
  mpu.setZGyroOffset(mpu6050_offsets[5]);

  printf("\n");
  printf("  ax \t ay \t az \t gx \t gy \t gz:\n");
  while (true) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    printf("  %d \t %d \t %d \t %d \t %d \t %d\r", ax, ay, az, gx, gy, gz);
    fflush(stdout);
    bcm2835_delay(100);
  }
  return 1;
}
