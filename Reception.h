#define MIN_ANGLE -30
#define MAX_ANGLE 30
#define MIN_SPEED_ROT -135
#define MAX_SPEED_ROT 135 // 90°/s

#define FLYING_MODE_ANGLE 0
#define FLYING_MODE_ACCRO 1

class Reception
{
  // Channel 1: Ailerons 1.09 to 1.90 ms
  // Channel 2: Prof 1.09 to 1.90 ms
  // Channel 3: Throttle 1.09 to 1.90 ms
  // Channel 4: Rudder 1.09 to 1.90 ms

private:
  // PWM computation
  bool initialized = false;
  int channel = 0;
  float PWM_Start = 0;
  float PWM_Stop = 0;
  float PWM_Width = 0;
  int cPPM[7] = {0, 0, 0, 0, 0, 0, 0}; // 6 channels plus separation
  
public:
  bool  IsReady() { return  initialized; };

  // Angle Mode:
  inline float GetAileronsAngle(){ return -(map(cPPM[0], 1080, 1900, MIN_ANGLE, MAX_ANGLE)); };
  inline float GetElevatorAngle(){ return -(map(cPPM[1], 1080, 1900, MIN_ANGLE, MAX_ANGLE)); };

  // Accro mode:
  inline float GetAileronsSpeed(){ return -(map(cPPM[0], 1080, 1900, MIN_SPEED_ROT, MAX_SPEED_ROT)); };
  inline float GetElevatorSpeed(){ return -(map(cPPM[1], 1080, 1900, MIN_SPEED_ROT, MAX_SPEED_ROT)); };
  
  inline int GetThrottle(){  return map(cPPM[2], 1080, 1900, MIN_POWER, MAX_POWER); };
  inline int GetRudder(){ return map(cPPM[3], 1080, 1900, -100, 100); };
  inline int GetFlyingMode(){ if (cPPM[4] > 1500) return FLYING_MODE_ACCRO; else return FLYING_MODE_ANGLE; }; //1900 inter H en bas, 1090 inter H en haut
  inline int GetGyroGain1(){ return cPPM[5]; };
  inline int GetGyroGain2(){ return cPPM[4]; };

  inline void GetWidth(void){
    PWM_Stop = micros();
    PWM_Width = PWM_Stop-PWM_Start;
    PWM_Start = PWM_Stop;
    
   if( initialized )
    cPPM[channel] = PWM_Width; 

    if( PWM_Width > 4000 ) // If delay more than 4ms
    {
      channel = 0;
      initialized = true;
    }else
      channel++;
   }
};
