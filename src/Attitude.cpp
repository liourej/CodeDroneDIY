#include <avr/wdt.h>
#include "Attitude.h"
#include "CheckIMU.h"
#include <limits.h>

#define AXIS_NB 3
#define SAMPLES_NB 10

#define RAD2DEG(angle) angle * 180 / PI

void Attitude::Init()
{
    // Initialize MPU 6050
    accelgyro.initialize();
    //  +-1000°s max  /!\ Be carrefull when changing this parameter:
    //  "GyroSensitivity" must be updated accordingly !!!
    accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
    //  +-8g max /!\ Be carrefull when changing this parameter:
    //  "AcceleroSensitivity" must be updated accordingly !!!
    accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
    wdt_reset();
    if (!accelgyro.testConnection())
        Serial.println(F("Test failed"));

    Serial.println(F("/********* IMU self-test *********/"));
    if (!checkIMU.All(accelgyro, AcceleroSensitivity))
        Serial.println(F("IMU SELF TEST FAILED !!!!!"));
    else
        Serial.println(F("IMU self test succeed"));
}

inline void Attitude::GetCorrectedAccelGyro(float _accMeasures[],
                                            float _gyroMeasures[])
{
    int16_t accel[AXIS_NB] = {0, 0, 0};
    int16_t speed[AXIS_NB] = {0, 0, 0};

    accelgyro.getMotion6(&accel[0], &accel[1], &accel[2], &speed[0], &speed[1], &speed[2]);

    // Correct raw data with offset
    for(int axis = 0; axis < AXIS_NB; axis++) {
      _accMeasures[axis] = static_cast<float>(accel[axis] - offset[axis]) / AcceleroSensitivity;
      _gyroMeasures[axis] = static_cast<float>(speed[axis] - offset[axis]) / GyroSensitivity;
    }
}

int16_t Attitude::ComputeDelta(int16_t _list[], int _size) {
  int16_t maxVal = 0;
  int16_t minVal = INT_MAX;

  for (int sample = 0; sample < _size; sample++) {
    if (_list[sample] > maxVal)
      maxVal = _list[axis];
    if (_list[sample] < minVal)
      minVal = _list[axis];
  }
  return (maxVal - minVal);
}

float Attitude::ComputeMean(int16_t _list[], int _size) {
  int16_t mean = 0;
  for (int sample = 0; sample < _size; sample++) {
    mean = mean + _list[sample];
  }
  return (mean/_size);
}
void Attitude::ComputeGyroOffsets() {
  int16_t gyroRawX[SAMPLES_NB];
  int16_t gyroRawY[SAMPLES_NB];
  int16_t gyroRawZ[SAMPLES_NB];
  
  int32_t offsetSum[AXIS_NB] = {0, 0, 0};

  int16_t maxVal[AXIS_NB] = {-32768, -32768, -32768};
    int16_t minVal[AXIS_NB] = {32767, 32767, 32767};
    int16_t delta[AXIS_NB] = {0, 0, 0};

    for (int axis = 0; axis < AXIS_NB; axis++)
        offset[axis] = 0;

    // mean on 10 samples during 2 sec
    for (int sample = 0; sample < 10; sample++) {
        accelgyro.getRotation(&gyroRawX[sample], &gyroRawY[sample], &gyroRawZ[samples]);
        }
        wdt_reset();
        delay(200);
    }

    // Compute mean and delta max
    for (int axis = 0; axis < AXIS_NB; axis++) {
        offset[axis] = ComputeMean(
        delta[axis] = maxVal[axis] - minVal[axis];
    }

    // Check deltas to be sure IMU does not move during mean computation
    for (int axis = 0; axis < AXIS_NB; axis++) {
        if (delta[axis] > (10 * GyroSensitivity)) // 10°/s max
            offsetComputed = false;
    }

    if (offsetComputed) {
        Serial.print(F("Gyroscope offsets Computed :"));
        for (int axis = 0; axis < AXIS_NB; axis++) {
            Serial.print(offset[axis] / GyroSensitivity);
            Serial.print(" ");
        }
        Serial.print("(deg.s-1) ");
        Serial.print("\n");
        return true;
    } else {
      Serial.println(F("ERROR DURING GYRO OFFSETS COMPUTATION !!"));
      return false;
    }
}

bool Attitude::ComputeAccelOffset() {
    int16_t accRaw[AXIS_NB] = {0, 0, 0};
    int32_t offsetSum[AXIS_NB] = {0, 0, 0};

    int16_t maxVal[AXIS_NB] = {-32768, -32768, -32768};
    int16_t minVal[AXIS_NB] = {32767, 32767, 32767};
    int16_t delta[AXIS_NB] = {0, 0, 0};

    for (int axis = 0; axis < AXIS_NB; axis++)
        offset[axis] = 0;

    // mean on 10 samples during 2 sec
    for (int sample = 0; sample < 10; sample++) {
        accelgyro.getAcceleration(&accRaw[0], &accRaw[1], &accRaw[2]);
        for (int coord = 0; coord < 6; coord++) {
            if (accRaw[coord] > maxVal[coord])
                maxVal[coord] = accRaw[coord];
            if (accRaw[coord] < minVal[coord])
                minVal[coord] = accRaw[coord];
            offsetSum[coord] = offsetSum[coord] + accRaw[coord];
        }
        wdt_reset();
        delay(200);
    }

    // Compute mean and delta max
    for (int axis = 0; axis < AXIS_NB; axis++) {
        offset[axis] = offsetSum[axis] / 10;
        delta[axis] = maxVal[axis] - minVal[axis];
    }

    // Check deltas to be sure IMU does not move during mean computation
    for (int axis = 0; axis < AXIS_NB; axis++) {
        if (delta[axis] > (0.2 * AcceleroSensitivity))
            offsetComputed = false;
    }

    // Zacc is gravity, it should be 1G ie 4096 LSB/g at -+8g sensitivity
    offset[2] = offset[2] - AcceleroSensitivity;

    if (offsetComputed) {
        Serial.print(F("Acceleration offsets Computed :"));
        for (int axis = 0; axis < AXIS_NB; axis++) {
            Serial.print(offset[axis] / AcceleroSensitivity);
            Serial.print(" ");
        }
        Serial.print("(m.s-2) ");
        return true;
    } else {
      Serial.println(F("ERROR DURING ACCELERATION OFFSETS COMPUTATION !!"));
      return false;
    }
}

// Compute accelerometer and gyroscope offsets
void Attitude::ComputeOffsets()
{
 if ( ComputeGyroOffsets() && ComputeAccelOffset())
   offsetComputed = true;
}

inline void Attitude::Normalize(float _acc[])
{
    float norm =
            sqrt(_acc[0] * _acc[0] + _acc[1] * _acc[1] + _acc[2] * _acc[2]);

    _acc[0] = _acc[0] / norm;
    _acc[1] = _acc[1] / norm;
    _acc[2] = _acc[2] / norm;
}

float Attitude::GetFilterTimeConstant(float _loopTimeSec)
{
    return ((HighPassFilterCoeff * _loopTimeSec) / (1 - HighPassFilterCoeff));
}

// Get position combining acc + gyro
void Attitude::GetCurrPos(float _pos[], float _speed[], float _loop_time)
{
    // float roll, pitch = 0;
    float accRaw[AXIS_NB] = {0, 0, 0};
    float gyroRaw[AXIS_NB] = {0, 0, 0};

    // Get corrected data from gyro and accelero
    GetCorrectedAccelGyro(accRaw, gyroRaw);

    // Compute rotation speed using gyroscopes
    _speed[0] = gyroRaw[0];
    _speed[1] = gyroRaw[1];
    _speed[2] = gyroRaw[2];

    Normalize(accRaw);

    // Use complementary filter to merge gyro and accelerometer data
    // High pass filter on gyro, and low pass filter on accelerometer
    _pos[0] =
            HighPassFilterCoeff * (_pos[0] + (gyroRaw[0]) * _loop_time)
            + (1 - HighPassFilterCoeff) * RAD2DEG(atan(accRaw[1] / accRaw[2]));
    _pos[1] =
            HighPassFilterCoeff * (_pos[1] + (gyroRaw[1]) * _loop_time)
            + (1 - HighPassFilterCoeff) * RAD2DEG(-atan(accRaw[0] / accRaw[2]));
}
