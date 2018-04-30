#include <avr/wdt.h>
#include "Stabilization.h"

void Stabilization::Init() {
  // MPU6050: join I2C bus
  Wire.begin();
  Wire.setClock(400000L);  // Communication with MPU-6050 at 400KHz

  while (!Rx.IsReady()) {
    Serial.println(F("Rx not ready, try again, please wait. "));
    ESCs.Idle();
    wdt_reset();
    delay(200);
  }
  // MPU6050, MS5611: initialize MPU6050 and MS5611 devices (IMU and barometer)

  Attitude.Init();

  if ((ESCs.MAX_POWER == 1860) && (ESCs.MAX_THROTTLE >= (1860 * 0.8)))
    Serial.println(F("!!!!!!!!!!!!!!!!!!!!FLYING MODE POWER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! "));
  else if ( (ESCs.MAX_POWER <= 1300) )
    Serial.println(F("DEBUG MODE POWER!!! "));
  else
    Serial.println(F("UNEXPECTED POWER "));

  Serial.print(F("MAX_POWER: "));
  Serial.print(ESCs.MAX_POWER);
  Serial.print(F(" MAX_THROTTLE_PERCENT: "));
  Serial.println(ESCs.MAX_THROTTLE_PERCENT);
  // Angle mode PID config
  rollPosPID_Angle.SetGains(anglePosPIDParams);
  pitchPosPID_Angle.SetGains(anglePosPIDParams);
  rollSpeedPID_Angle.SetGains(angleSpeedPIDParams);
  pitchSpeedPID_Angle.SetGains(angleSpeedPIDParams);

  // Adjust Kp from potentiometer on A0
  yawSpeedPIDParams[1] = map(analogRead(0), 0, 1023, 0, 500);
  Serial.print("Yaw kP: ");
  Serial.println(yawSpeedPIDParams[1]);
  yawSpeedPID_Angle.SetGains(yawSpeedPIDParams);

  // Accro mode PID config
  rollSpeedPID_Accro.SetGains(accroSpeedPIDParams);
  pitchSpeedPID_Accro.SetGains(accroSpeedPIDParams);
  yawSpeedPID_Accro.SetGains(yawSpeedPIDParams);
}

void Stabilization::Accro(float _loopTimeSec, Reception &_Rx) {
  Attitude.GetCurrPos(posCurr, speedCurr, _loopTimeSec);
  rollMotorPwr = rollSpeedPID_Accro.ComputeCorrection(_Rx.GetAileronsSpeed(), speedCurr[0],
        _loopTimeSec);
    pitchMotorPwr = pitchSpeedPID_Accro.ComputeCorrection(_Rx.GetElevatorSpeed(), speedCurr[1],
        _loopTimeSec);
    yawMotorPwr = yawSpeedPID_Accro.ComputeCorrection(_Rx.GetRudder(), speedCurr[2],
        _loopTimeSec);
}

void Stabilization::Angle(float _loopTimeSec, Reception &_Rx) {
  Attitude.GetCurrPos(posCurr, speedCurr, _loopTimeSec);
  stateMachine.throttleWasHigh = true;
    rollPosCmd = rollPosPID_Angle.ComputeCorrection(_Rx.GetAileronsAngle(), posCurr[0],
        _loopTimeSec);
    rollMotorPwr = rollSpeedPID_Angle.ComputeCorrection(rollPosCmd, speedCurr[0],
        _loopTimeSec);

    pitchPosCmd = pitchPosPID_Angle.ComputeCorrection(_Rx.GetElevatorAngle(), posCurr[1],
        _loopTimeSec);
    pitchMotorPwr = pitchSpeedPID_Angle.ComputeCorrection(pitchPosCmd, speedCurr[1],
        _loopTimeSec);

    yawMotorPwr = yawSpeedPID_Angle.ComputeCorrection(_Rx.GetRudder(), speedCurr[2],
        _loopTimeSec);
}

void Stabilization::PrintAccroModeParameters() {
 Serial.println(F("/********* PID settings *********/"));
    rollSpeedPID_Accro.PrintGains();
    pitchSpeedPID_Accro.PrintGains();
    yawSpeedPID_Accro.PrintGains();
}

void Stabilization::PrintAngleModeParameters() {
  Serial.println(F("/********* PID settings *********/"));
  rollPosPID_Angle.PrintGains();
  pitchPosPID_Angle.PrintGains();

  rollSpeedPID_Angle.PrintGains();
  pitchSpeedPID_Angle.PrintGains();
  yawSpeedPID_Angle.PrintGains();
  Serial.println(F("/********* Complementary filter *********/"));
  Serial.print("Coefficient: ");
  Serial.print(Attitude.HighPassFilterCoeff);
  Serial.print(" Time constant: ");
  Serial.println(Attitude.GetFilterTimeConstant(0.00249));
}

void Stabilization::ResetPID(int *_rollMotorPwr, int *_pitchMotorPwr, int *_yawMotorPwr) {
  *_pitchMotorPwr = *_rollMotorPwr = *_yawMotorPwr = 0;  // No correction if throttle put to min
  rollPosPID_Angle.Reset();
  pitchPosPID_Angle.Reset();
  rollSpeedPID_Angle.Reset();
  pitchSpeedPID_Angle.Reset();
  yawSpeedPID_Angle.Reset();
  rollSpeedPID_Accro.Reset();
  pitchSpeedPID_Accro.Reset();
  yawSpeedPID_Accro.Reset();
}
