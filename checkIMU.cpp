#include "MPU6050.h"
#include "checkIMU.h"

float ComputeAccFactoryTrimValue(float _accTestVal) {
  if ( _accTestVal == 0 )
    return 0;
  else
    //       (4096 * 0.34)*(pow( (0.92/0.34) , (((float)selfTest[0] - 1.0)/30.0))); // FT[Xa] factory trim calculation
    return (4096 * 0.34 * pow(0.92,          ((_accTestVal - 1) / (pow(2, 5) - 2))) / 0.34);
}

float ComputeGyroFactoryTrimValue(float _gyroTestVal, bool _isYcoord) {
  if ( _gyroTestVal == 0 )
    return 0;
  else if ( _isYcoord )
    return (-25 * 131 * pow(1.046, _gyroTestVal - 1));
  else
    return (25 * 131 * pow(1.046, _gyroTestVal - 1));
}

bool CheckGyro(MPU6050 _accelgyro) {
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
  _accelgyro.getMotion6(&dataTestDisabled[0],&dataTestDisabled[1], &dataTestDisabled[2], &dataTestDisabled[3], &dataTestDisabled[4], &dataTestDisabled[5]);

  I2Cdev::writeBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_XG_ST_BIT, true);
  I2Cdev::writeBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_XG_ST_BIT, true);
  I2Cdev::writeBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_XG_ST_BIT, true);

  delay(500);
  //_Position.GetAccelGyro(_accelgyro, dataTestEnabled);
  uint8_t rawData[4];
  //_Position.GetAccelGyro(_accelgyro, dataTestEnabled);
  rawData[0] = _accelgyro.getAccelXSelfTestVal(); // X-axis self-test results
  rawData[1] = _accelgyro.getAccelYSelfTestVal(); // Y-axis self-test results
  rawData[2] = _accelgyro.getAccelZSelfTestVal(); // Z-axis self-test results
  rawData[3] = _accelgyro.getAccelMixedSelfTestVal();

  //Serial.print("rawData:\t"); Serial.print(rawData[0]); Serial.print("\t"); Serial.print(rawData[1]); Serial.print("\t"); Serial.print(rawData[2]); Serial.print("\t"); Serial.println(rawData[3]);


  // Extract the gyration test results first
  dataTestEnabled[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit unsigned integer
  dataTestEnabled[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit unsigned integer
  dataTestEnabled[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit unsigned integer

 // Serial.print("dataTestEnabled:\t"); Serial.print(dataTestEnabled[3]); Serial.print("\t"); Serial.print(dataTestEnabled[4]); Serial.print("\t"); Serial.println(dataTestEnabled[5]);

  for (int axis = 3; axis < 6; axis ++) { // Gyro data
    if ( axis == 4) // if axis is Y, computation is different (negative sign)
      FT[axis] = ComputeGyroFactoryTrimValue( dataTestEnabled[axis], true );
    else
      FT[axis] = ComputeGyroFactoryTrimValue( dataTestEnabled[axis], false );
    STR[axis] = dataTestEnabled[axis] - dataTestDisabled[axis];
    trimChange[axis] = 100 + (( STR[axis] - FT[axis]) / FT[axis]) * 100;
    if ( abs(trimChange[axis]) > 14 ) // 14% See PS-MPU-6000A-00 rev 3.3 - �6.2 Accelerometer specifications
      testSucceed = false;
  }
  //Serial.print("FT:\t"); Serial.print(FT[3]); Serial.print("\t"); Serial.print(FT[4]); Serial.print("\t"); Serial.println(FT[5]);
  //Serial.print("STR:\t"); Serial.print(STR[3]); Serial.print("\t"); Serial.print(STR[4]); Serial.print("\t"); Serial.println(STR[5]);
  Serial.print("Gyro trim test results: "); Serial.print(trimChange[3]); Serial.print(" "); Serial.print(trimChange[4]); Serial.print(" "); Serial.println(trimChange[5]);

  // Restore normal operating
  I2Cdev::writeBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_XG_ST_BIT, false);
  I2Cdev::writeBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_XG_ST_BIT, false);
  I2Cdev::writeBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_XG_ST_BIT, false);
  _accelgyro.setFullScaleGyroRange(storeGyroSensitivity);

  return testSucceed;
}

bool CheckAccelero(MPU6050 _accelgyro, const float _acceleroSensitivity) {
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
  _accelgyro.getMotion6(&dataTestDisabled[0],&dataTestDisabled[1], &dataTestDisabled[2], &dataTestDisabled[3], &dataTestDisabled[4], &dataTestDisabled[5]);
  dataTestDisabled[2] = dataTestDisabled[2] - _acceleroSensitivity; // Remove gravity
  _accelgyro.setAccelXSelfTest(true);
  _accelgyro.setAccelYSelfTest(true);
  _accelgyro.setAccelZSelfTest(true);

  delay(500);
  uint8_t rawData[4];
  rawData[0] = _accelgyro.getAccelXSelfTestVal(); // X-axis self-test results
  rawData[1] = _accelgyro.getAccelYSelfTestVal(); // Y-axis self-test results
  rawData[2] = _accelgyro.getAccelZSelfTestVal(); // Z-axis self-test results
  rawData[3] = _accelgyro.getAccelMixedSelfTestVal();

 // Serial.print("rawData:\t"); Serial.print(rawData[0]); Serial.print("\t"); Serial.print(rawData[1]); Serial.print("\t"); Serial.print(rawData[2]); Serial.print("\t"); Serial.println(rawData[3]);

  // Extract the acceleration test results first
  dataTestEnabled[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit unsigned integer
  dataTestEnabled[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 4 ; // YA_TEST result is a five-bit unsigned integer
  dataTestEnabled[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) >> 4 ; // ZA_TEST result is a five-bit unsigned integer

  //Serial.print("dataTestEnabled:\t"); Serial.print(dataTestEnabled[0]); Serial.print("\t"); Serial.print(dataTestEnabled[1]); Serial.print("\t"); Serial.println(dataTestEnabled[2]);
  //Serial.print("dataTestDisabled:\t"); Serial.print(dataTestDisabled[0]); Serial.print("\t"); Serial.print(dataTestDisabled[1]); Serial.print("\t"); Serial.println(dataTestDisabled[2]);
  for (int axis = 0; axis < 3; axis ++) {
    FT[axis] = ComputeAccFactoryTrimValue( dataTestEnabled[axis] );
    STR[axis] = dataTestEnabled[axis] - dataTestDisabled[axis];

    trimChange[axis] = 100 + (( STR[axis] - FT[axis]) / FT[axis]) * 100;
    if ( abs(trimChange[axis]) > 14 ) // 14% See PS-MPU-6000A-00 rev 3.3 - �6.2 Accelerometer specifications
      testSucceed = false;
  }

//  Serial.print("FT:\t"); Serial.print(FT[0]); Serial.print("\t"); Serial.print(FT[1]); Serial.print("\t"); Serial.println(FT[2]);
//  Serial.print("STR:\t"); Serial.print(STR[0]); Serial.print("\t"); Serial.print(STR[1]); Serial.print("\t"); Serial.println(STR[2]);

  Serial.print("Acc trim test results: "); Serial.print(trimChange[0]); Serial.print(" "); Serial.print(trimChange[1]); Serial.print(" "); Serial.println(trimChange[2]);

  // Restore normal operating
  _accelgyro.setAccelXSelfTest(false);
  _accelgyro.setAccelYSelfTest(false);
  _accelgyro.setAccelZSelfTest(false);
  _accelgyro.setFullScaleAccelRange(storeAccSensitivity);
  delay(500);

  return testSucceed;
}

bool CheckIMU(MPU6050 _accelgyro, const float _AcceleroSensitivity) {
  bool testSucceed = true;

  // Check accelerometers
  if ( !CheckAccelero( _accelgyro, _AcceleroSensitivity) )
    testSucceed = false;

  // Check Gyroscopes
  if ( !CheckGyro(_accelgyro) )
     testSucceed = false;

  return testSucceed;
}
