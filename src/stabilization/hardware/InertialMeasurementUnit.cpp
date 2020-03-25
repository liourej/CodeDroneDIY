#include <avr/wdt.h>
#include "InertialMeasurementUnit.h"

void InertialMeasurementUnit::Init() {
    // Initialize MPU 6050
    accelgyro.initialize();

    SetGyroRange(MPU6050_GYRO_FS_1000);
    SetAccRange(MPU6050_ACCEL_FS_8);

    wdt_reset();
    if (!accelgyro.testConnection())
        Serial.println(F("Test failed"));
}

inline void InertialMeasurementUnit::GetCorrectedAccelGyro(float _accMeasures[], float _gyroMeasures[]) {
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

void InertialMeasurementUnit::SetAccRange(uint8_t _range) {
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

void InertialMeasurementUnit::SetGyroRange(uint8_t _range) {
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
void InertialMeasurementUnit::ComputeOffsets() {
    if (ComputeGyroOffsets() && ComputeAccelOffsets())
        offsetComputed = true;
}

bool InertialMeasurementUnit::ComputeGyroOffsets() {
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

bool InertialMeasurementUnit::ComputeAccelOffsets() {
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


