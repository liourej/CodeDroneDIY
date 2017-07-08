// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

class GetPosition
{
  public:
    float HighPassFilterCoeff = 0.98;
    const float AcceleroSensitivity = 4096; // LSB/g at -+8g sensitivity
    const float GyroSensitivity = 32.8; // LSB/°/s  250=>131 or 500=>65.5 or 1000=>32.8 or 2000=>16.4

    bool offsetComputed = false;
    int16_t offset[6] = {0, 0, 0, 0, 0, 0};

  private:
    void GetCorrectedAccelGyro(MPU6050 _accelgyro, float _accMeasures[], float _gyroMeasures[]);
    void GetCorrectedGyro(MPU6050 _gyro, float _data[]);
    void Normalize( float _acc[] );

  public:
    bool AreOffsetComputed(void) {
      return offsetComputed;
    }
    void ComputeOffsets(MPU6050 _accelgyro);
    void GetCurrPos(MPU6050 _accelgyro, float _pos[], float _speed[], float _loop_time);
    void GetCurrSpeed(MPU6050 _accelgyro, float speedCurr[]);
};

