#include <iostream>
using namespace std;

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

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

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 * bno;

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  cout << "Orientation Sensor Raw Data Test" << endl;
  cout << endl;

  /* Initialise the sensor */
  if(!bno->begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    cout << "Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!";
    while(1);
  }

  usleep(1000 * 1000);

  /* Display the current temperature */
  int8_t temp = bno->getTemp();
  cout << "Current Temperature: ";
  cout << temp;
  cout << " C" << endl;
  cout << endl;

  bno->setExtCrystalUse(true);

  cout << "Calibration status values: 0=uncalibrated, 3=fully calibrated" << endl;
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> euler = bno->getVector(Adafruit_BNO055::VECTOR_EULER);

  /* Display the floating point data */
  cout << "X: ";
  cout << euler.x();
  cout << " Y: ";
  cout << euler.y();
  cout << " Z: ";
  cout << euler.z();
  cout << "\t\t";

  /*
  // Quaternion data
  imu::Quaternion quat = bno->getQuat();
  cout << "qW: ";
  cout << quat.w(), 4;
  cout << " qX: ";
  cout << quat.x(), 4;
  cout << " qY: ";
  cout << quat.y(), 4;
  cout << " qZ: ";
  cout << quat.z(), 4;
  cout << "\t\t";
  */

  /* Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;
  bno->getCalibration(&system, &gyro, &accel, &mag);
  cout << "CALIBRATION: Sys=";
  cout << system, DEC;
  cout << " Gyro=";
  cout << gyro, DEC;
  cout << " Accel=";
  cout << accel, DEC;
  cout << " Mag=";
  cout << mag, DEC << endl;

  usleep(BNO055_SAMPLERATE_DELAY_MS * 1000);
}


 
int main(int argc, char *argv[])
{
  bno = Adafruit_BNO055(-1, 0x29, 1);
  setup();
  for(int i = 0; i < 10; i++) loop();
  return 0;
}