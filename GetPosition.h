// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

class GetPosition
{
private:
  const float HighPassFilterCoeff = 0.98;
  const float LowPassFilterCoeff = 0.02;
  const float GyroSensitivity = 65.5; // 131 or 65.5 or 32.8 or 16.4
  
  float offset[6] = {0, 0, 0, 0, 0, 0};

private:
  void GetAccelGyro(MPU6050 _accelgyro, float _data[]);
  void GetCorrectedAccelGyro(MPU6050 _accelgyro, float _data[]);
  void Normalize( float _acc[] );

public:    
  void ComputeOffsets(MPU6050 _accelgyro);
  void GetCurrPos(MPU6050 _accelgyro, float _pos[], float _speed[], float _loop_time);
  void GetCurrSpeed(MPU6050 _accelgyro, float speedCurr[]);
};

