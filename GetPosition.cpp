#include <avr/wdt.h>
#include "GetPosition.h"
#include "checkIMU.h"

void GetPosition::Init() {
  // Initialize MS5611 sensor (barometer for altitude)
  while (!ms5611.begin(MS5611_ULTRA_HIGH_RES))
    delay(500);

  accelgyro.initialize();
  accelgyro.setFullScaleGyroRange( MPU6050_GYRO_FS_1000); //  +-1000°s max  /!\ Be carrefull when changing this parameter: "GyroSensitivity" must be updated accordingly !!!
  accelgyro.setFullScaleAccelRange( MPU6050_ACCEL_FS_8 );//  +-8g max /!\ Be carrefull when changing this parameter: "AcceleroSensitivity" must be updated accordingly !!!
  wdt_reset();
  if ( !accelgyro.testConnection())
    Serial.println(F("Test failed"));

  Serial.println(F("/********* IMU self-test *********/"));
  if ( !CheckIMU(accelgyro, AcceleroSensitivity) )
    Serial.println(F("IMU SELF TEST FAILED !!!!!"));
  else
    Serial.println(F("IMU self test succeed"));

  ms5611.refreshTemperature();
}

inline void GetPosition::GetCorrectedAccelGyro(float _accMeasures[], float _gyroMeasures[])
{
  int16_t ax, ay, az, gx, gy, gz = 0;

  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);   // 2ms !!

  // Correct raw data with offset
  _accMeasures[0] = (float)(ax - offset[0] ) / AcceleroSensitivity;
  _accMeasures[1] = (float)(ay - offset[1] ) / AcceleroSensitivity;
  _accMeasures[2] = (float)(az - offset[2] ) / AcceleroSensitivity;
  _gyroMeasures[0] = (float)(gx - offset[3] ) / GyroSensitivity;
  _gyroMeasures[1] = (float)(gy - offset[4] ) / GyroSensitivity;
  _gyroMeasures[2] = (float)(gz - offset[5] ) / GyroSensitivity;

  //Normalize(_data); // Normalize 3 first array cases (acceleration)
}

inline void GetPosition::GetCorrectedGyro(float _data[])
{
  int16_t gx, gy, gz;

  accelgyro.getRotation(&gx, &gy, &gz);   // 2ms !!

  // Correct raw data with offset
  _data[0] = (float)(gx - offset[3] ) / GyroSensitivity;
  _data[1] = (float)(gy - offset[4] ) / GyroSensitivity;
  _data[2] = (float)(gz - offset[5] ) / GyroSensitivity;
}

// Compute accelerometer and gyroscope offsets
void GetPosition::ComputeOffsets()
{
  int16_t accGyroRaw[6] = {0, 0, 0, 0, 0, 0};
  int32_t offsetSum[6] = {0, 0, 0, 0, 0, 0};

  int16_t maxVal[6] = { -32768, -32768, -32768, -32768, -32768, -32768};
  int16_t minVal[6] = {32767, 32767, 32767, 32767, 32767, 32767};
  int16_t delta[6] = {0, 0, 0, 0, 0, 0};

  for (int i = 0; i < 6; i++)
    offset[i] = 0;

  // mean on 10 samples during 2 sec
  for (int sample = 0; sample < 10; sample++)
  {
    accelgyro.getMotion6(&accGyroRaw[0], &accGyroRaw[1], &accGyroRaw[2], &accGyroRaw[3], &accGyroRaw[4], &accGyroRaw[5]);
    for ( int coord = 0; coord < 6; coord++) {
      if ( accGyroRaw[coord] > maxVal[coord])
        maxVal[coord] = accGyroRaw[coord];
      if ( accGyroRaw[coord] < minVal[coord])
        minVal[coord] = accGyroRaw[coord];
      offsetSum[coord] = offsetSum[coord] + accGyroRaw[coord];
      //Serial.print(accGyroRaw[coord]); Serial.print("\t");
    }
    //Serial.print("\n");
    wdt_reset();
    delay(200);
  }

  // Compute mean and delta max
  for (int i = 0; i < 6; i++) {
    offset[i] =  offsetSum[i] / 10;
    delta[i] = maxVal[i] - minVal[i];
  }

  // Check deltas to be sure IMU does not move during mean computation
  offsetComputed = true;
  for (int acc = 0; acc < 3; acc++) {
    if ( delta[acc] > (0.2 * AcceleroSensitivity))
      offsetComputed = false;
  }

  for (int gyro = 3; gyro < 6; gyro++) {
    if ( delta[gyro] > (10 * GyroSensitivity) ) // 10°/s max
      offsetComputed = false;
  }

  // Zacc is gravity, it should be 1G ie 4096 LSB/g at -+8g sensitivity
  offset[2] = offset[2] - AcceleroSensitivity;



  if (offsetComputed) {
    Serial.print(F("Offsets Computed :"));
    for (int i = 0; i < 6; i++) {
      Serial.print(offset[i]); Serial.print("\t");
    }
    Serial.print("\n");
  } else
    Serial.println(F("ERROR DURING OFFSETS COMPUTATION !!"));
};

inline void GetPosition::Normalize( float _acc[] )
{
  float norm = sqrt( _acc[0] * _acc[0] + _acc[1] * _acc[1] + _acc[2] * _acc[2] );

  _acc[0] = _acc[0] / norm;
  _acc[1] = _acc[1] / norm;
  _acc[2] = _acc[2] / norm;
}

bool GetPosition::IsVectorNormalized( float _acc[], float _epsilon ) {
  float norm = sqrt( _acc[0] * _acc[0] + _acc[1] * _acc[1] + _acc[2] * _acc[2] );

  if ( abs(1 - norm) < _epsilon )
    return true;

  return false;
}

float GetPosition::PercentVectorNormalized( float _acc[]) {
  float norm = sqrt( _acc[0] * _acc[0] + _acc[1] * _acc[1] + _acc[2] * _acc[2] );
  return abs(1 - norm);
}

// Get rotation speed using only gyro
void GetPosition::GetCurrSpeed(float _speed[])
{
  // float roll, pitch = 0;
  float gyroRaw[3] = {0, 0, 0};

  // Get corrected data from gyro and accelero
  GetCorrectedGyro(gyroRaw);

  _speed[0] = gyroRaw[0];
  _speed[1] = gyroRaw[1];
  _speed[2] = gyroRaw[2];
}

float GetPosition::GetFilterTimeConstant(float _loopTimeSec) {
  return ( (HighPassFilterCoeff * _loopTimeSec) / (1 - HighPassFilterCoeff));
}

// Get position combining acc + gyro
void GetPosition::GetCurrPos(float _pos[], float _speed[], float _loop_time)
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
  _pos[0] = HighPassFilterCoeff * (_pos[0] + (gyroRaw[0]) * _loop_time) + (1 - HighPassFilterCoeff) * ((atan(accRaw[1] / accRaw[2])) * 57.2957795130823); // High pass filter on gyro, and low pass filter on accelerometer
  _pos[1] = HighPassFilterCoeff * (_pos[1] + (gyroRaw[1]) * _loop_time) + (1 - HighPassFilterCoeff) * ((-atan(accRaw[0] / accRaw[2])) * 57.2957795130823); // High pass filter on gyro, and low pass filter on accelerometer

}

float GetPosition::GetVerticalSpeed(void) {
  long realPressure = 0;
  static bool initialized = false;
  static float altiPrev = 0.0;
  float altiCurr = 0.0;
  static float measures[10];
  static int indice = 0;
  float mean = 0.0;
  float verticalSpeed = 0.0;

  realPressure = ms5611.readPressureFast();

  // Compute altitude mean
  measures[indice] = ms5611.getAltitude(realPressure);
  indice++;

  if ( indice > 9) {
    indice = 0;
    initialized = true;
  }

  if ( initialized == false)
    return 0.0;

  // Compute mean altitude
  mean = 0.0;
  for (int i = 0; i < 10; i++)
    mean = mean + measures[indice];

  // Compute vertical speed
  altiCurr = mean / 10;
  verticalSpeed = altiCurr - altiPrev;
  altiPrev = altiCurr;

  return verticalSpeed;
}
