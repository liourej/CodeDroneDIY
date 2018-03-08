#include "PID.h"
// PID setup
const float GAIN = 0.010;

// Angle mode
float anglePosPIDParams[4] = {0.010, 268, 0.5, 0.0};  // G, Kp, Kd, Ki
float angleSpeedPIDParams[4] = {0.010, 192, 0.0, 0.0};
float altiSpeedPIDParams[4] = {0.010, 10, 0.0, 0.0};

// Accro mode
// 450mm frame, 10x4.5" bi-pale - Tested during flight test: OK
float accroSpeedPIDParams[4] = { 0.010, 192, 0.0, 0.0};
// 220 mm frame, 5x4.5" bi-pale - Flight test FAILED!
// float accroSpeedPIDParams[4] = {0.08, 80.0, 30.0, 0.5};

float ACCRO_YAW_KP = 0;  // Not used for now

// Yaw PID
float yawSpeedPIDParams[4] = { 0.010, 150.0, 0.0, 0.0};  // G, Kp, Kd, Ki

void PID::SetGains(float _params[4]) {
  G = _params[0];
  Kp = _params[1];
  Kd = _params[2];
  Ki = _params[3];
}

void PID::Reset() {
  errorPrev = 0;
  integrator = 0;
}

int PID::ComputeCorrection(float _cmd, float _pos, float _loopTime) {
  float correction = 0;
  error = _cmd - _pos;
  integrator = integrator + error;
  correction = G * (Kp * error +  Kd * ((error - errorPrev) / (_loopTime)) + Ki * integrator);

  errorPrev = error;

  // Correction in us
  return correction;
}

void PID::PrintGains(void) {
  Serial.print("G: ");
  Serial.print(G);
  Serial.print(" Kp: ");
  Serial.print(Kp);
  Serial.print(" Kd: ");
  Serial.print(Kd);
  Serial.print(" Ki: ");
  Serial.println(Ki);
}
