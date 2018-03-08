#ifndef CHECKIMU_H_
#define CHECKIMU_H_

#define SELF_TEST_X      0x0D
#define SELF_TEST_Y      0x0E
#define SELF_TEST_Z      0x0F
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define PWR_MGMT_1       0x6B  // Device defaults to the SLEEP mode

#define MPU6050_GCONFIG_XG_ST_BIT 7
#define MPU6050_GCONFIG_YG_ST_BIT 6
#define MPU6050_GCONFIG_ZG_ST_BIT 5
#define MPU6050_RA_GYRO_CONFIG 0x1B
#define MPU6050_ADDRESS_AD0_LOW     0x68  // address pin low (GND),
// default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69  // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW

float ComputeAccFactoryTrimValue(float _accTestVal);
float ComputeGyroFactoryTrimValue(float _gyroTestVal, bool _isYcoord);
bool CheckGyro(MPU6050 _accelgyro);
bool CheckAccelero(MPU6050 _accelgyro, const float _AcceleroSensitivity);
bool CheckIMU(MPU6050 _accelgyro, const float _AcceleroSensitivity);

#endif  // CHECKIMU_H_
