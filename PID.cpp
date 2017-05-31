#include "PID.h"
#include "Settings.h"

void PID::SetPIDCoef(float _G, float _Kp, float _Kd, float _Ki) {
  G = _G;
  Kp = _Kp;
  Kd = _Kd;
  Ki = _Ki;
};

void PID::Reset() {
  errorPrev = 0;
  integrator = 0;
};

int PID::GetPIDOutput( float _cmd, float _pos, float _loopTime ) {
  float correction = 0;
  error = _cmd - _pos;
  integrator = integrator + error;
  correction = G * (Kp * error +  Kd * (( error - errorPrev) / (_loopTime)) + Ki * integrator);

  errorPrev = error;

  // Correction in us
  return correction;
};
