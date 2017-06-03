#include "PID.h"

void PID::SetGains(float _params[4]) {
  G = _params[0];
  Kp = _params[1];
  Kd = _params[2];
  Ki = _params[3];
};

void PID::Reset() {
  errorPrev = 0;
  integrator = 0;
};

int PID::ComputeCorrection( float _cmd, float _pos, float _loopTime ) {
  float correction = 0;
  error = _cmd - _pos;
  integrator = integrator + error;
  correction = G * (Kp * error +  Kd * (( error - errorPrev) / (_loopTime)) + Ki * integrator);

  errorPrev = error;

  // Correction in us
  return correction;
};

void PID::PrintGains(void) {
  Serial.print("G: "); Serial.print(G); Serial.print("\tKp: "); Serial.print(Kp); Serial.print("\tKd: "); Serial.print(Kd); Serial.print("\tKi: "); Serial.println(Ki);
};
