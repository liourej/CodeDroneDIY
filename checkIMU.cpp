#include "MPU6050.h"
#include "GetPosition.h"
#include "checkIMU.h"

float ComputeAccFactoryTrimValue(float _accTestVal) {
  if ( _accTestVal == 0 )
    return 0;
  else
    return (4096 * 0.34 * pow(0.92, ( (_accTestVal - 1) / (pow(2, 5) - 2))) / 0.34);
}

float ComputeGyroFactoryTrimValue(float _gyroTestVal, bool _isYcoord) {
  if ( _gyroTestVal == 0 )
    return 0;
  else if ( _isYcoord )
    return (-25 * 131 * pow(1.046, _gyroTestVal - 1));
  else
    return (25 * 131 * pow(1.046, _gyroTestVal - 1));
}

bool CheckGyro(MPU6050 _accelgyro, GetPosition _Position) {
  int16_t dataTestEnabled[6] = {0, 0, 0, 0, 0, 0};
  int16_t dataTestDisabled[6] = {0, 0, 0, 0, 0, 0};
  float STR[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  float FT[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  float trimChange[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  bool testSucceed = true;
  int storeGyroSensitivity = -1;

  storeGyroSensitivity = _accelgyro.getFullScaleGyroRange();
  _accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  
   delay(1000);
   _accelgyro.getMotion6(&dataTestDisabled[0], &dataTestDisabled[1], &dataTestDisabled[2], &dataTestDisabled[3], &dataTestDisabled[4], &dataTestDisabled[5]);
  
  I2Cdev::writeBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_XG_ST_BIT, true);
  I2Cdev::writeBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_XG_ST_BIT, true);
  I2Cdev::writeBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_XG_ST_BIT, true);

  delay(1000);
  _accelgyro.getMotion6(&dataTestEnabled[0], &dataTestEnabled[1], &dataTestEnabled[2], &dataTestEnabled[3], &dataTestEnabled[4], &dataTestEnabled[5]);
  for (int axis = 3; axis < 6; axis ++) { // Gyro data
    if ( axis == 4) // if axis is Y, computation is different (negative sign)
      FT[axis] = ComputeGyroFactoryTrimValue( dataTestEnabled[axis], true );
    else
      FT[axis] = ComputeGyroFactoryTrimValue( dataTestEnabled[axis], false );
    STR[axis] = dataTestEnabled[axis] - dataTestDisabled[axis];
    trimChange[axis] = (( STR[axis] - FT[axis]) / FT[axis]) * 100;
    if ( abs(trimChange[axis]) > 14 ) // 14% See PS-MPU-6000A-00 rev 3.3 - �6.2 Accelerometer specifications
      testSucceed = false;
  }
  Serial.print("Gyro trim test results:\t"); Serial.print(trimChange[3]); Serial.print("\t"); Serial.print(trimChange[4]); Serial.print("\t"); Serial.println(trimChange[5]);

  // Restore normal operating
  I2Cdev::writeBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_XG_ST_BIT, false);
  I2Cdev::writeBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_XG_ST_BIT, false);
  I2Cdev::writeBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_XG_ST_BIT, false);
  _accelgyro.setFullScaleGyroRange(storeGyroSensitivity);
  delay(1000);
}

bool CheckAccelero(MPU6050 _accelgyro, GetPosition _Position) {
  int16_t dataTestEnabled[6] = {0, 0, 0, 0, 0, 0};
  int16_t dataTestDisabled[6] = {0, 0, 0, 0, 0, 0};
  float STR[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  float FT[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  float trimChange[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  bool testSucceed = true;
  int storeAccSensitivity = -1;
  
  storeAccSensitivity = _accelgyro.getFullScaleAccelRange();
  _accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);

  delay(1000);
  _accelgyro.getMotion6(&dataTestDisabled[0], &dataTestDisabled[1], &dataTestDisabled[2], &dataTestDisabled[3], &dataTestDisabled[4], &dataTestDisabled[5]);

  _accelgyro.setAccelXSelfTest(true);
  _accelgyro.setAccelYSelfTest(true);
  _accelgyro.setAccelZSelfTest(true);

  delay(1000);
   _accelgyro.getMotion6(&dataTestEnabled[0], &dataTestEnabled[1], &dataTestEnabled[2], &dataTestEnabled[3], &dataTestEnabled[4], &dataTestEnabled[5]);

  for (int axis = 0; axis < 3; axis ++) {
    FT[axis] = ComputeAccFactoryTrimValue( dataTestEnabled[axis] );
    STR[axis] = dataTestEnabled[axis] - dataTestDisabled[axis];
    trimChange[axis] = (( STR[axis] - FT[axis]) / FT[axis]) * 100;
    if ( abs(trimChange[axis]) > 14 ) // 14% See PS-MPU-6000A-00 rev 3.3 - �6.2 Accelerometer specifications
      testSucceed = false;
  }
  Serial.print("Acc trim test results:\t"); Serial.print(trimChange[0]); Serial.print("\t"); Serial.print(trimChange[1]); Serial.print("\t"); Serial.println(trimChange[2]);

  // Restore normal operating
  _accelgyro.setAccelXSelfTest(false);
  _accelgyro.setAccelYSelfTest(false);
  _accelgyro.setAccelZSelfTest(false);
  _accelgyro.setFullScaleAccelRange(storeAccSensitivity);
  delay(1000);
}

bool CheckIMU(MPU6050 _accelgyro, GetPosition _Position) {
  bool testSucceed = true;
  
  // Check accelerometers
  if ( !CheckAccelero( _accelgyro, _Position) )
    testSucceed = false;

  // Check Gyroscopes
  if ( !CheckGyro(_accelgyro, _Position) )
    testSucceed = false;

  return testSucceed;
}
