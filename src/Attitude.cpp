#include <avr/wdt.h>
#include "Attitude.h"
#include "CheckIMU.h"

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
    /*  if (!CheckIMU(accelgyro, AcceleroSensitivity))
        Serial.println(F("IMU SELF TEST FAILED !!!!!"));
      else
        Serial.println(F("IMU self test succeed"));
    */
}

inline void Attitude::GetCorrectedAccelGyro(float _accMeasures[],
                                            float _gyroMeasures[])
{
    int16_t ax, ay, az, gx, gy, gz = 0;

    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Correct raw data with offset
    _accMeasures[0] = static_cast<float>(ax - offset[0]) / AcceleroSensitivity;
    _accMeasures[1] = static_cast<float>(ay - offset[1]) / AcceleroSensitivity;
    _accMeasures[2] = static_cast<float>(az - offset[2]) / AcceleroSensitivity;
    _gyroMeasures[0] = static_cast<float>(gx - offset[3]) / GyroSensitivity;
    _gyroMeasures[1] = static_cast<float>(gy - offset[4]) / GyroSensitivity;
    _gyroMeasures[2] = static_cast<float>(gz - offset[5]) / GyroSensitivity;
}

// Compute accelerometer and gyroscope offsets
void Attitude::ComputeOffsets()
{
    int16_t accGyroRaw[6] = {0, 0, 0, 0, 0, 0};
    int32_t offsetSum[6] = {0, 0, 0, 0, 0, 0};

    int16_t maxVal[6] = {-32768, -32768, -32768, -32768, -32768, -32768};
    int16_t minVal[6] = {32767, 32767, 32767, 32767, 32767, 32767};
    int16_t delta[6] = {0, 0, 0, 0, 0, 0};

    for (int i = 0; i < 6; i++)
        offset[i] = 0;

    // mean on 10 samples during 2 sec
    for (int sample = 0; sample < 10; sample++) {
        accelgyro.getMotion6(&accGyroRaw[0], &accGyroRaw[1], &accGyroRaw[2],
                             &accGyroRaw[3], &accGyroRaw[4], &accGyroRaw[5]);
        for (int coord = 0; coord < 6; coord++) {
            if (accGyroRaw[coord] > maxVal[coord])
                maxVal[coord] = accGyroRaw[coord];
            if (accGyroRaw[coord] < minVal[coord])
                minVal[coord] = accGyroRaw[coord];
            offsetSum[coord] = offsetSum[coord] + accGyroRaw[coord];
        }
        wdt_reset();
        delay(200);
    }

    // Compute mean and delta max
    for (int i = 0; i < 6; i++) {
        offset[i] = offsetSum[i] / 10;
        delta[i] = maxVal[i] - minVal[i];
    }

    // Check deltas to be sure IMU does not move during mean computation
    offsetComputed = true;
    for (int acc = 0; acc < 3; acc++) {
        if (delta[acc] > (0.2 * AcceleroSensitivity))
            offsetComputed = false;
    }

    for (int gyro = 3; gyro < 6; gyro++) {
        if (delta[gyro] > (10 * GyroSensitivity)) // 10°/s max
            offsetComputed = false;
    }

    // Zacc is gravity, it should be 1G ie 4096 LSB/g at -+8g sensitivity
    offset[2] = offset[2] - AcceleroSensitivity;

    if (offsetComputed) {
        Serial.print(F("Offsets Computed :"));
        for (int i = 0; i < 3; i++) {
            Serial.print(offset[i] / AcceleroSensitivity);
            Serial.print(" ");
        }
        Serial.print("(m.s-2) ");
        for (int i = 3; i < 6; i++) {
            Serial.print(offset[i] / GyroSensitivity);
            Serial.print(" ");
        }
        Serial.print("(deg.s-1) ");
        Serial.print("\n");
    } else {
        Serial.println(F("ERROR DURING OFFSETS COMPUTATION !!"));
    }
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
    float accRaw[3] = {0, 0, 0};
    float gyroRaw[3] = {0, 0, 0};

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
