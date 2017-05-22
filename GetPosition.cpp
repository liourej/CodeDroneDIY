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

inline void GetPosition::GetCorrectedAccelGyro(MPU6050 _accelgyro, float _data[])
{
  int16_t ax, ay, az, gx, gy, gz;

  _accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);   // 2ms !!
  
  // Correct raw data with offset
  //for( int i= 0; i < 6; i++)
  _data[0] = (ax - offset[0] );
  _data[1] = (ay - offset[1] );
  _data[2] = (az - offset[2] );
  _data[3] = (gx - offset[3] );
  _data[4] = (gy - offset[4] );
  _data[5] = (gz - offset[5] );

    Normalize(_data); // Normalize 3 first array cases (acceleration)
}

// Compute accelerometer and gyroscope offsets
void GetPosition::ComputeOffsets(MPU6050 _accelgyro)
{
  float accGyroRaw[6]={0, 0, 0, 0, 0, 0};
  double offsetSum[6] = {0, 0, 0, 0, 0, 0};
  
  for(int i=0; i < 6; i++)
   offset[i] = 0;
  
 // mean on 10 samples
  for(int sample=0; sample < 10; sample++)
  {
     GetAccelGyro(_accelgyro, accGyroRaw);
     for( int coord = 0; coord < 6; coord++)
      offsetSum[coord] = offsetSum[coord] + accGyroRaw[coord];
     //Serial.print("Sample:\t");
    // printCoord(accGyroRaw);
     wdt_reset();
     delay(5);
  }
  for(int i=0; i< 6; i++)
   offset[i] =  offsetSum[i]/10;

  // Compute scale to 9.81m.s-2 on z
  //accScale =  offset[2]/9.81;
  offset[2] = 0;
  
  Serial.println("Offsets Computed");
 // printCoord(_offsets);
 }
 
inline void GetPosition::Normalize( float _acc[] )
{
   float norm = sqrt( _acc[0]*_acc[0] + _acc[1]*_acc[1] + _acc[2]*_acc[2] );
  
  _acc[0] = _acc[0]/norm;
  _acc[1] = _acc[1]/norm;
  _acc[2] = _acc[2]/norm;
}

// Get rotation speed using only gyro
void GetPosition::GetCurrSpeed(const MPU6050 _accelgyro, float _speed[])
{
 // float roll, pitch = 0;
  float accGyroRaw[6]={0, 0, 0, 0, 0, 0};
    
  // Get corrected data from gyro and accelero
  GetCorrectedAccelGyro(_accelgyro, accGyroRaw);

   _speed[0] = accGyroRaw[0+3]/GyroSensitivity;
   _speed[1] = accGyroRaw[1+3]/GyroSensitivity;
   _speed[2] = accGyroRaw[2+3]/GyroSensitivity;
}

// Get position combining acc + gyro
void GetPosition::GetCurrPos(const MPU6050 _accelgyro, float _pos[], const float _loop_time)
{
 // float roll, pitch = 0;
	float accGyroRaw[6]={0, 0, 0, 0, 0, 0};
	  
  // Get corrected data from gyro and accelero
  GetCorrectedAccelGyro(_accelgyro, accGyroRaw);
 
  // Use complementary filter to merge gyro and accelerometer data
  _pos[0] = HighPassFilterCoeff*(_pos[0] + (accGyroRaw[0+3]/GyroSensitivity)*_loop_time) + LowPassFilterCoeff*((atan(accGyroRaw[1]/accGyroRaw[2]))*57.2957795130823);  // High pass filter on gyro, and low pass filter on accelerometer
  _pos[1] = HighPassFilterCoeff*(_pos[1] + (accGyroRaw[1+3]/GyroSensitivity)*_loop_time) + LowPassFilterCoeff*((atan(accGyroRaw[0]/accGyroRaw[2]))*57.2957795130823);  // High pass filter on gyro, and low pass filter on accelerometer
}
