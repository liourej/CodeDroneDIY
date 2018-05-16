#ifndef CHECKIMU_H_
#define CHECKIMU_H_
#include "MPU6050.h"

#define MPU6050_GCONFIG_XG_ST_BIT 7

class CheckIMU {
  private:
    float ComputeAccFactoryTrimValue(float _accTestVal);
    float ComputeGyroFactoryTrimValue(float _gyroTestVal, bool _isYcoord);
    bool Gyro(MPU6050 &_accelgyro);
    bool Accelero(MPU6050 &_accelgyro, const float _AcceleroSensitivity);
    void SetAccelSelfTest(MPU6050 &_accelgyro, const bool _enable);

  public:
    bool All(MPU6050 &_accelgyro, const float _AcceleroSensitivity);
};
#endif // CHECKIMU_H_
