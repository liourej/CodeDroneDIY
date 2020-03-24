#ifndef ATTITUDE_H_
#define ATTITUDE_H_

#include "Wire.h"
#include "../libraries/I2Cdev/I2Cdev.h"
#include "../libraries/MPU6050/MPU6050.h"
#include "../customLibs/Math.h"

class Attitude : public Math {
  public:
    /* /!\ HighPassFilterCoeff is an important coeff for complementary filter
      /!\
      Too high, position will drift, to low, there will be noise from
      accelerometer
      14-Jul-2017: loop time = 2.49ms. Gyro does not drift with coef =< 0.999
      timeCste = (coeff*dt)/(1-coeff)
       coeff = timeCste/(dt + timeCste) If we want 0.5sec, coeff = 0.5/(0.003 +
      0.5) = 0.994
    */
    float HighPassFilterCoeff = 0.9995;

  private:
    static const int AXIS_NB = 3;
    static const int SAMPLES_NB = 10;
    float AcceleroSensitivity = -1;
    float GyroSensitivity = -1;
    int16_t gyroOffsets[AXIS_NB] = {0, 0, 0};
    int16_t accOffsets[AXIS_NB] = {0, 0, 0};
    bool initialized = false;
    bool offsetComputed = false;
    MPU6050 accelgyro; // IMU

  private:
    void GetCorrectedAccelGyro(float _accMeasures[], float _gyroMeasures[]);
    void Normalize(float _acc[]);
    bool ComputeGyroOffsets();
    bool ComputeAccelOffsets();
    void SetAccRange(uint8_t _range);
    void SetGyroRange(uint8_t _range);

  public:
    void Init();
    float GetFilterTimeConstant(float _loopTime);
    bool AreOffsetComputed(void) {
        return offsetComputed;
    }
    void ComputeOffsets();
    void GetCurrPos(float _pos[], float _speed[], float _loop_time);
};

#endif // ATTITUDE_H_
