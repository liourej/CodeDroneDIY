#include <avr/wdt.h>
#include "InertialMeasurementUnit.h"
void InertialMeasurementUnit::Init() {
    // MPU6050: join I2C bus
    Wire.begin();
    Wire.setClock(400000L); // Communication with MPU-6050 at 400KHz

    // Initialize MPU 6050
    accelgyro.initialize();

    SetGyroRange(MPU6050_GYRO_FS_1000);
    SetAccRange(MPU6050_ACCEL_FS_8);

    wdt_reset();
    if (!accelgyro.testConnection())
        CustomSerialPrint::println(F("InertialMeasurementUnit: Function testConnection failed"));
}

void InertialMeasurementUnit::GetCorrectedAccelGyro(float _accMeasures[], float _gyroMeasures[]) {
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
        AcceleroSensitivity = 16384;
        break;
    case MPU6050_ACCEL_FS_4:
        AcceleroSensitivity = 8192;
        break;
    case MPU6050_ACCEL_FS_8:
        AcceleroSensitivity = 4096;
        break;
    case MPU6050_ACCEL_FS_16:
        AcceleroSensitivity = 2048;
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
    else
        offsetComputed = false;
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
        CustomSerialPrint::print(gyroRaw[0][sample]);
        CustomSerialPrint::print("\t");
        CustomSerialPrint::print(gyroRaw[1][sample]);
        CustomSerialPrint::print("\t");
        CustomSerialPrint::println(gyroRaw[2][sample]);
        delay(200);
    }

    // Compute mean
    for (int axis = 0; axis < AXIS_NB; axis++) {
        if (!CustomMath::ComputeMean(gyroRaw[axis], SAMPLES_NB, (10 * GyroSensitivity), &mean)) {
            CustomSerialPrint::println(F("ERROR DURING SPEED OFFSETS COMPUTATION !!"));
            return false;
        }
        gyroOffsets[axis] = static_cast<int16_t>(mean);
    }

    CustomSerialPrint::print(F("Gyroscope offsets Computed :"));
    for (int axis = 0; axis < AXIS_NB; axis++) {
        CustomSerialPrint::print(gyroOffsets[axis] / GyroSensitivity);
        CustomSerialPrint::print(" ");
    }
    CustomSerialPrint::println("(deg.s-1) ");
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
        CustomSerialPrint::print(accRaw[0][sample]);
        CustomSerialPrint::print("\t");
        CustomSerialPrint::print(accRaw[1][sample]);
        CustomSerialPrint::print("\t");
        CustomSerialPrint::println(accRaw[2][sample]);
        wdt_reset();
        delay(200);
    }

    // Mean computation
    for (int axis = 0; axis < AXIS_NB; axis++) {
        if (!CustomMath::ComputeMean(accRaw[axis], SAMPLES_NB, (0.2 * AcceleroSensitivity),
                                     &mean)) {
            CustomSerialPrint::println(F("ERROR DURING ACCELERATION OFFSETS COMPUTATION !!"));
            return false;
        }
        accOffsets[axis] = static_cast<int16_t>(mean);
    }

    // Zacc is gravity, it should be 1G ie 4096 LSB/g at -+8g sensitivity
    accOffsets[2] = accOffsets[2] - AcceleroSensitivity;

    CustomSerialPrint::print(F("Acceleration offsets Computed :"));
    for (int axis = 0; axis < AXIS_NB; axis++) {
        CustomSerialPrint::print(accOffsets[axis] / AcceleroSensitivity);
        CustomSerialPrint::print(" ");
    }
    CustomSerialPrint::print("(m.s-2) ");
    return true;
}