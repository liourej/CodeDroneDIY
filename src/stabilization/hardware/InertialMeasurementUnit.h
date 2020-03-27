#ifndef INERTIALMEASUREMENTUNIT_H_
#define INERTIALMEASUREMENTUNIT_H_

#include "Wire.h"
#include "../../libraries/I2Cdev/I2Cdev.h"
#include "../../libraries/MPU6050/MPU6050.h"
#include "../../customLibs/CustomMath.h"

class InertialMeasurementUnit : public CustomMath {
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
    bool ComputeGyroOffsets();
    bool ComputeAccelOffsets();
    void SetAccRange(uint8_t _range);
    void SetGyroRange(uint8_t _range);

  public:
    void Init();
    bool AreOffsetComputed(void) {
        return offsetComputed;
    }
    void ComputeOffsets();
    void GetCorrectedAccelGyro(float _accMeasures[], float _gyroMeasures[]);
};

#endif // INERTIALMEASUREMENTUNIT_H_