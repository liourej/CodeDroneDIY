#include <avr/wdt.h>
#include "GetPosition.h"

void GetPosition::GetAccelGyro(MPU6050 _accelgyro, float _data[])
{
  int16_t ax, ay, az, gx, gy, gz;
  _accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  _data[0] = ax;
  _data[1] = ay;
  _data[2] = az;
  _data[3] = gx;
  _data[4] = gy;
  _data[5] = gz;
}

inline void GetPosition::GetCorrectedAccelGyro(MPU6050 _accelgyro, float _accMeasures[], float _gyroMeasures[])
{
  int16_t ax, ay, az, gx, gy, gz;

  _accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);   // 2ms !!

  // Correct raw data with offset
  //for( int i= 0; i < 6; i++)
  _accMeasures[0] = (ax - offset[0] ) / AcceleroSensitivity;
  _accMeasures[1] = (ay - offset[1] ) / AcceleroSensitivity;
  _accMeasures[2] = (az - offset[2] ) / AcceleroSensitivity;
  _gyroMeasures[0] = (gx - offset[3] ) / GyroSensitivity;
  _gyroMeasures[1] = (gy - offset[4] ) / GyroSensitivity;
  _gyroMeasures[2] = (gz - offset[5] ) / GyroSensitivity;

  //Normalize(_data); // Normalize 3 first array cases (acceleration)
}

inline void GetPosition::GetCorrectedGyro(MPU6050 _gyro, float _data[])
{
  int16_t gx, gy, gz;

  _gyro.getRotation(&gx, &gy, &gz);   // 2ms !!

  // Correct raw data with offset
  _data[0] = (gx - offset[3] );
  _data[1] = (gy - offset[4] );
  _data[2] = (gz - offset[5] );
}

// Compute accelerometer and gyroscope offsets
void GetPosition::ComputeOffsets(MPU6050 _accelgyro)
{
  float accGyroRaw[6] = {0, 0, 0, 0, 0, 0};
  double offsetSum[6] = {0, 0, 0, 0, 0, 0};

  for (int i = 0; i < 6; i++)
    offset[i] = 0;

  // mean on 10 samples
  for (int sample = 0; sample < 10; sample++)
  {
    GetAccelGyro(_accelgyro, accGyroRaw);
    for ( int coord = 0; coord < 6; coord++)
      offsetSum[coord] = offsetSum[coord] + accGyroRaw[coord];
    //Serial.print("Sample:\t");
    // printCoord(accGyroRaw);
    wdt_reset();
    delay(5);
  }
  for (int i = 0; i < 6; i++)
    offset[i] =  offsetSum[i] / 10;

  // Zacc is gravity, it should be 1G ie 4096 LSB/g at -+8g sensitivity
  offset[2] = offset[2] - AcceleroSensitivity;

  Serial.println(F("Offsets Computed"));
  // printCoord(_offsets);
  offsetComputed = true;
}

/*inline void GetPosition::Normalize( float _acc[] )
  {
  float norm = sqrt( _acc[0] * _acc[0] + _acc[1] * _acc[1] + _acc[2] * _acc[2] );

  _acc[0] = _acc[0] / norm;
  _acc[1] = _acc[1] / norm;
  _acc[2] = _acc[2] / norm;
  }*/

bool IsVectorNormalized( float _acc[], float _epsilon ) {
  float norm = sqrt( _acc[0] * _acc[0] + _acc[1] * _acc[1] + _acc[2] * _acc[2] );

  if ( abs(1-norm) < _epsilon )
    return true;

  return false;
}

// Get rotation speed using only gyro
void GetPosition::GetCurrSpeed(MPU6050 _accelgyro, float _speed[])
{
  // float roll, pitch = 0;
  float gyroRaw[3] = {0, 0, 0};

  // Get corrected data from gyro and accelero
  GetCorrectedGyro(_accelgyro, gyroRaw);

  _speed[0] = gyroRaw[0] / GyroSensitivity;
  _speed[1] = gyroRaw[1] / GyroSensitivity;
  _speed[2] = gyroRaw[2] / GyroSensitivity;
}

// Get position combining acc + gyro
void GetPosition::GetCurrPos(MPU6050 _accelgyro, float _pos[], float _speed[], float _loop_time)
{
  // float roll, pitch = 0;
  float accRaw[3] = {0, 0, 0};
  float gyroRaw[3] = {0, 0, 0};

  // Get corrected data from gyro and accelero
  GetCorrectedAccelGyro(_accelgyro, accRaw, gyroRaw);

  // Compute rotation speed using gyroscopes
  _speed[0] = gyroRaw[0] / GyroSensitivity;
  _speed[1] = gyroRaw[1] / GyroSensitivity;
  _speed[2] = gyroRaw[2] / GyroSensitivity;

  // If accereleration norm is > 1g, device is moving, do not use acceleration data
  if ( IsVectorNormalized(accRaw, 0.05) )
    HighPassFilterCoeff = 0.98;
  else 
    HighPassFilterCoeff = 1.0;
  
  // Use complementary filter to merge gyro and accelerometer data
  _pos[0] = HighPassFilterCoeff * (_pos[0] + (gyroRaw[0] / GyroSensitivity) * _loop_time) + (1-HighPassFilterCoeff) * ((atan(accRaw[1] / accRaw[2])) * 57.2957795130823); // High pass filter on gyro, and low pass filter on accelerometer
  _pos[1] = HighPassFilterCoeff * (_pos[1] + (gyroRaw[1] / GyroSensitivity) * _loop_time) + (1-HighPassFilterCoeff) * ((atan(accRaw[0] / accRaw[2])) * 57.2957795130823); // High pass filter on gyro, and low pass filter on accelerometer
}
