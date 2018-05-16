#include "CheckIMU.h"
#include "MPU6050.h"

bool CheckIMU::All(MPU6050 &_accelgyro, const float _AcceleroSensitivity) {
    bool testSucceed = true;

    // Check accelerometers
    if (!Accelero(_accelgyro, _AcceleroSensitivity))
        testSucceed = false;

    // Check Gyroscopes
    if (!Gyro(_accelgyro))
        testSucceed = false;

    return testSucceed;
}

bool CheckIMU::Gyro(MPU6050 &_accelgyro) {
    float dataTestEnabled[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    int16_t dataTestDisabled[6] = {0, 0, 0, 0, 0, 0};
    float STR[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float FT[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float trimChange[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    bool testSucceed = true;
    int storeGyroSensitivity = -1;

    storeGyroSensitivity = _accelgyro.getFullScaleGyroRange();
    _accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

    delay(500);
    _accelgyro.getMotion6(&dataTestDisabled[0], &dataTestDisabled[1], &dataTestDisabled[2],
                          &dataTestDisabled[3], &dataTestDisabled[4], &dataTestDisabled[5]);

    I2Cdev::writeBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_XG_ST_BIT,
                     true);
    I2Cdev::writeBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_XG_ST_BIT,
                     true);
    I2Cdev::writeBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_XG_ST_BIT,
                     true);

    delay(500);
    uint8_t rawData[3];
    rawData[0] = _accelgyro.getAccelXSelfTestFactoryTrim(); // X-axis self-test results
    rawData[1] = _accelgyro.getAccelYSelfTestFactoryTrim(); // Y-axis self-test results
    rawData[2] = _accelgyro.getAccelZSelfTestFactoryTrim(); // Z-axis self-test results

    // Extract the gyration test results first
    dataTestEnabled[3] = rawData[0] & 0x1F; // XG_TEST result is a five-bit
                                            // unsigned integer
    dataTestEnabled[4] = rawData[1] & 0x1F; // YG_TEST result is a five-bit
                                            // unsigned integer
    dataTestEnabled[5] = rawData[2] & 0x1F; // ZG_TEST result is a five-bit
                                            // unsigned integer

    for (int axis = 3; axis < 6; axis++) { // Gyro data
        if (axis == 4)                     // If axis is Y, computation is different (negative sign)
            FT[axis] = ComputeGyroFactoryTrimValue(dataTestEnabled[axis], true);
        else
            FT[axis] = ComputeGyroFactoryTrimValue(dataTestEnabled[axis], false);
        STR[axis] = dataTestEnabled[axis] - dataTestDisabled[axis];
        trimChange[axis] = 100 + ((STR[axis] - FT[axis]) / FT[axis]) * 100;
        if (abs(trimChange[axis]) > 14) // 14% See PS-MPU-6000A-00 rev 3.3
                                        // 6.2 Accelerometer specifications
            testSucceed = false;
    }
    Serial.print("Gyro trim test results: ");
    Serial.print(trimChange[3]);
    Serial.print(" ");
    Serial.print(trimChange[4]);
    Serial.print(" ");
    Serial.println(trimChange[5]);

    // Restore normal operating
    I2Cdev::writeBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_XG_ST_BIT,
                     false);
    I2Cdev::writeBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_XG_ST_BIT,
                     false);
    I2Cdev::writeBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_XG_ST_BIT,
                     false);
    _accelgyro.setFullScaleGyroRange(storeGyroSensitivity);

    return testSucceed;
}

bool CheckIMU::Accelero(MPU6050 &_accelgyro, const float _acceleroSensitivity) {
    float dataTestEnabled[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    int16_t dataTestDisabled[6] = {0, 0, 0, 0, 0, 0};
    float STR[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float FT[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    float trimChange[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    bool testSucceed = true;
    int storeAccSensitivity = -1;

    storeAccSensitivity = _accelgyro.getFullScaleAccelRange();
    _accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);

    delay(500);
    _accelgyro.getMotion6(&dataTestDisabled[0], &dataTestDisabled[1], &dataTestDisabled[2],
                          &dataTestDisabled[3], &dataTestDisabled[4], &dataTestDisabled[5]);
    dataTestDisabled[2] = dataTestDisabled[2] - _acceleroSensitivity; // Remove
                                                                      // gravity
    SetAccelSelfTest(_accelgyro, true);

    delay(500);
    uint8_t rawData[3];
    rawData[0] = _accelgyro.getAccelXSelfTestFactoryTrim(); // X-axis self-test results
    rawData[1] = _accelgyro.getAccelYSelfTestFactoryTrim(); // Y-axis self-test results
    rawData[2] = _accelgyro.getAccelZSelfTestFactoryTrim(); // Z-axis self-test results

    // Extract the acceleration test results first
    // XA_TEST result is a five-bit unsigned integer
    dataTestEnabled[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4;
    // YA_TEST result is a five-bit unsigned integer
    dataTestEnabled[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 4;
    // ZA_TEST result is a five-bit unsigned integer
    dataTestEnabled[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) >> 4;

    for (int axis = 0; axis < 3; axis++) {
        FT[axis] = ComputeAccFactoryTrimValue(dataTestEnabled[axis]);
        STR[axis] = dataTestEnabled[axis] - dataTestDisabled[axis];

        trimChange[axis] = 100 + ((STR[axis] - FT[axis]) / FT[axis]) * 100;
        // 14% See PS-MPU-6000A-00 rev 3.3 - 6.2 Accelerometer specifications
        if (abs(trimChange[axis]) > 14)
            testSucceed = false;
    }

    Serial.print("Acc trim test results: ");
    Serial.print(trimChange[0]);
    Serial.print(" ");
    Serial.print(trimChange[1]);
    Serial.print(" ");
    Serial.println(trimChange[2]);

    // Restore normal operating
    SetAccelSelfTest(_accelgyro, false);
    _accelgyro.setFullScaleAccelRange(storeAccSensitivity);
    delay(500);

    return testSucceed;
}

void CheckIMU::SetAccelSelfTest(MPU6050 &_accelgyro, const bool _enable) {
    _accelgyro.setAccelXSelfTest(_enable);
    _accelgyro.setAccelYSelfTest(_enable);
    _accelgyro.setAccelZSelfTest(_enable);
}

float CheckIMU::ComputeAccFactoryTrimValue(float _accTestVal) {
    if (_accTestVal == 0)
        return 0;
    else
        return (4096 * 0.34 * pow(0.92, ((_accTestVal - 1) / (pow(2, 5) - 2))) / 0.34);
}

float CheckIMU::ComputeGyroFactoryTrimValue(float _gyroTestVal, bool _isYcoord) {
    if (_gyroTestVal == 0)
        return 0;
    else if (_isYcoord)
        return (-25 * 131 * pow(1.046, _gyroTestVal - 1));
    else
        return (25 * 131 * pow(1.046, _gyroTestVal - 1));
}
