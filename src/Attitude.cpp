#include <avr/wdt.h>
#include "Attitude.h"
#include "CheckIMU.h"

void Attitude::Init() {
    // Initialize MPU 6050
    accelgyro.initialize();

    SetGyroRange(MPU6050_GYRO_FS_1000);
    SetAccRange(MPU6050_ACCEL_FS_8);

    wdt_reset();
    if (!accelgyro.testConnection())
        Serial.println(F("Test failed"));

    Serial.println(F("/********* IMU self-test *********/"));
    if (!checkIMU.All(accelgyro, AcceleroSensitivity))
        Serial.println(F("IMU SELF TEST FAILED !!!!!"));
    else
        Serial.println(F("IMU self test succeed"));
}

inline void Attitude::GetCorrectedAccelGyro(float _accMeasures[], float _gyroMeasures[]) {
    int16_t accel[AXIS_NB] = {0, 0, 0};
    int16_t speed[AXIS_NB] = {0, 0, 0};

    accelgyro.getMotion6(&accel[0], &accel[1], &accel[2], &speed[0], &speed[1], &speed[2]);

    // Correct raw data with offset
    for (int axis = 0; axis < AXIS_NB; axis++) {
        _accMeasures[axis] =
                static_cast<float>((accel[axis] - accOffsets[axis]) / AcceleroSensitivity);
        _gyroMeasures[axis] =
                static_cast<float>((speed[axis] - gyroOffsets[axis]) / GyroSensitivity);
    }
}

void Attitude::SetAccRange(uint8_t _range) {
    switch (_range) {
    case MPU6050_ACCEL_FS_2:
        GyroSensitivity = 16384;
        break;
    case MPU6050_ACCEL_FS_4:
        GyroSensitivity = 8192;
        break;
    case MPU6050_ACCEL_FS_8:
        GyroSensitivity = 4096;
        break;
    case MPU6050_ACCEL_FS_16:
        GyroSensitivity = 2048;
        break;
    }
    accelgyro.setFullScaleAccelRange(_range);
}

void Attitude::SetGyroRange(uint8_t _range) {
    switch (_range) {
    case MPU6050_GYRO_FS_250:
        GyroSensitivity = 131;
        break;
    case MPU6050_GYRO_FS_500:
        GyroSensitivity = 65.5;
        break;
    case MPU6050_GYRO_FS_1000:
        GyroSensitivity = 32.8;
        break;
    case MPU6050_GYRO_FS_2000:
        GyroSensitivity = 16.4;
        break;
    }
    accelgyro.setFullScaleGyroRange(_range);
}

// Compute accelerometer and gyroscope offsets
void Attitude::ComputeOffsets() {
    if (ComputeGyroOffsets() && ComputeAccelOffsets())
        offsetComputed = true;
}

bool Attitude::ComputeGyroOffsets() {
    int16_t gyroRaw[AXIS_NB][SAMPLES_NB];
    float mean = 0;

    for (int axis = 0; axis < AXIS_NB; axis++)
        for (int sample = 0; sample < 10; sample++)
            gyroRaw[axis][sample] = 0;

    // Get 10 samples during 2 sec
    for (int sample = 0; sample < 10; sample++) {
        accelgyro.getRotation(&gyroRaw[0][sample], &gyroRaw[1][sample], &gyroRaw[2][sample]);
        wdt_reset();
        delay(200);
    }

    // Compute mean
    for (int axis = 0; axis < AXIS_NB; axis++) {
        if (ComputeMean(gyroRaw[axis], SAMPLES_NB, (10 * GyroSensitivity), &mean) != 0)
            return false;
        gyroOffsets[axis] = static_cast<int16_t>(mean);
    }

    Serial.print(F("Gyroscope offsets Computed :"));
    for (int axis = 0; axis < AXIS_NB; axis++) {
        Serial.print(gyroOffsets[axis] / GyroSensitivity);
        Serial.print(" ");
    }
    Serial.println("(deg.s-1) ");
    return true;
}

bool Attitude::ComputeAccelOffsets() {
    int16_t accRaw[AXIS_NB][SAMPLES_NB];
    float mean = 0.0;

    for (int axis = 0; axis < AXIS_NB; axis++)
        for (int sample = 0; sample < 10; sample++)
            accRaw[axis][sample] = 0;

    // Get 10 samples during 2 sec
    for (int sample = 0; sample < 10; sample++) {
        accelgyro.getAcceleration(&accRaw[0][sample], &accRaw[1][sample], &accRaw[2][sample]);
        wdt_reset();
        delay(200);
    }

    // Mean computation
    for (int axis = 0; axis < AXIS_NB; axis++) {
        if (ComputeMean(accRaw[axis], SAMPLES_NB, (0.2 * AcceleroSensitivity), &mean))
            return false;
        accOffsets[axis] = static_cast<int16_t>(mean);
    }

    // Zacc is gravity, it should be 1G ie 4096 LSB/g at -+8g sensitivity
    accOffsets[2] = accOffsets[2] - AcceleroSensitivity;

    if (offsetComputed) {
        Serial.print(F("Acceleration offsets Computed :"));
        for (int axis = 0; axis < AXIS_NB; axis++) {
            Serial.print(accOffsets[axis] / AcceleroSensitivity);
            Serial.print(" ");
        }
        Serial.print("(m.s-2) ");
        return true;
    } else {
        Serial.println(F("ERROR DURING ACCELERATION OFFSETS COMPUTATION !!"));
        return false;
    }
}

inline void Attitude::Normalize(float _acc[]) {
    float norm = sqrt(_acc[0] * _acc[0] + _acc[1] * _acc[1] + _acc[2] * _acc[2]);

    _acc[0] = _acc[0] / norm;
    _acc[1] = _acc[1] / norm;
    _acc[2] = _acc[2] / norm;
}

float Attitude::GetFilterTimeConstant(float _loopTimeSec) {
    return ((HighPassFilterCoeff * _loopTimeSec) / (1 - HighPassFilterCoeff));
}

// Get position combining acc + gyro
void Attitude::GetCurrPos(float _pos[], float _speed[], float _loop_time) {
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
    _pos[0] = HighPassFilterCoeff * (_pos[0] + (gyroRaw[0]) * _loop_time)
              + (1 - HighPassFilterCoeff) * RAD2DEG(atan(accRaw[1] / accRaw[2]));
    _pos[1] = HighPassFilterCoeff * (_pos[1] + (gyroRaw[1]) * _loop_time)
              + (1 - HighPassFilterCoeff) * RAD2DEG(-atan(accRaw[0] / accRaw[2]));
}
