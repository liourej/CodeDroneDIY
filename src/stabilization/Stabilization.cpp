#include <avr/wdt.h>
#include "Stabilization.h"

void Stabilization::Init() {
    motorsSpeedControl.Init();

    // MPU6050: join I2C bus
    Wire.begin();
    Wire.setClock(400000L); // Communication with MPU-6050 at 400KHz

    while (!radioReception.IsReady()) {
        Serial.println(F("radioReception not ready, try again, please wait. "));
        motorsSpeedControl.Idle();
        wdt_reset();
        delay(200);
    }

    inertialMeasurementUnit.Init();

    if ((motorsSpeedControl.GetMotorsMaxPower() == 1860)
        && (motorsSpeedControl.GetMotorsMaxThrottle() >= (1860 * 0.8)))
        Serial.println(
                F("!!!!!!!!!!!!!!!!!!!!FLYING MODE "
                  "POWER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! "));
    else if ((motorsSpeedControl.GetMotorsMaxPower() <= 1300))
        Serial.println(F("DEBUG MODE POWER!!! "));
    else
        Serial.println(F("UNEXPECTED POWER "));

    Serial.print(F("MAX_POWER: "));
    Serial.print(motorsSpeedControl.GetMotorsMaxPower());
    Serial.print(F(" MAX_THROTTLE_PERCENT: "));
    Serial.println(motorsSpeedControl.GetMotorsMaxThrottlePercent());

    SetAngleModeControlLoopConfig();
    SetAccroModeControlLoopConfig();
    SetYawControlLoopConfig();
}

void Stabilization::SetAngleModeControlLoopConfig() {
    rollPosPID_Angle.SetGains(ControlLoopConstants::GetInstance()->anglePos);
    pitchPosPID_Angle.SetGains(ControlLoopConstants::GetInstance()->anglePos);
    rollSpeedPID_Angle.SetGains(ControlLoopConstants::GetInstance()->angleSpeed);
    pitchSpeedPID_Angle.SetGains(ControlLoopConstants::GetInstance()->angleSpeed);
}

void Stabilization::SetAccroModeControlLoopConfig() {
    rollSpeedPID_Accro.SetGains(ControlLoopConstants::GetInstance()->accroSpeed);
    pitchSpeedPID_Accro.SetGains(ControlLoopConstants::GetInstance()->accroSpeed);
}

void Stabilization::SetYawControlLoopConfig() {
    // Adjust Kp from potentiometer on A0
    ControlLoopConstants::GetInstance()->yawSpeed[1] = map(analogRead(0), 0, 1023, 0, 500);
    Serial.print("Yaw kP: ");
    Serial.println(ControlLoopConstants::GetInstance()->yawSpeed[1]);
    yawControlLoop.SetGains(ControlLoopConstants::GetInstance()->yawSpeed);
}

void Stabilization::Accro(float _loopTimeSec) {
    // Get current attitude (roll, pitch, yaw speeds)
    ComputeAttitude(angularPosCurr, angularSpeedCurr, _loopTimeSec);

    // Compute new speed command for each axis
    rollMotorPwr = rollSpeedPID_Accro.ComputeCorrection(radioReception.GetAileronsSpeed(),
                                                        angularSpeedCurr[XAXIS], _loopTimeSec);
    pitchMotorPwr = pitchSpeedPID_Accro.ComputeCorrection(radioReception.GetElevatorSpeed(),
                                                          angularSpeedCurr[YAXIS], _loopTimeSec);
    yawMotorPwr = yawControlLoop.ComputeCorrection(radioReception.GetRudder(), angularSpeedCurr[ZAXIS],
                                                      _loopTimeSec);

    // Apply computed speed command to motors
    SetMotorsPwrXConfig();
}

void Stabilization::Angle(float _loopTimeSec) {
    // Get current attitude (roll, pitch, yaw angles and speeds)
    ComputeAttitude(angularPosCurr, angularSpeedCurr, _loopTimeSec);

    // Compute roll position command
    int rollPosCmd = rollPosPID_Angle.ComputeCorrection(radioReception.GetAileronsAngle(),
                                                        angularPosCurr[XAXIS], _loopTimeSec);

    // Compute roll speed command
    rollMotorPwr =
            rollSpeedPID_Angle.ComputeCorrection(rollPosCmd, angularSpeedCurr[XAXIS], _loopTimeSec);

    // Compute pitch position command
    int pitchPosCmd = pitchPosPID_Angle.ComputeCorrection(radioReception.GetElevatorAngle(),
                                                          angularPosCurr[YAXIS], _loopTimeSec);

    // Compute pitch speed command
    pitchMotorPwr = pitchSpeedPID_Angle.ComputeCorrection(pitchPosCmd, angularSpeedCurr[YAXIS],
                                                          _loopTimeSec);

    // Compute yaw speed command
    yawMotorPwr = yawControlLoop.ComputeCorrection(radioReception.GetRudder(), angularSpeedCurr[ZAXIS],
                                                      _loopTimeSec);

    // Apply computed command to motors
    SetMotorsPwrXConfig();
}

float Stabilization::GetFilterTimeConstant(float _loopTimeSec) {
    return ((HighPassFilterCoeff * _loopTimeSec) / (1 - HighPassFilterCoeff));
}

// Compute attitude (pitch angle & speed and roll angle & speed) combining acc + gyro
void Stabilization::ComputeAttitude(float _angularPos[], float _angularSpeed[], float _loop_time) {
    float accRaw[nbAxis] = {0, 0, 0};
    float gyroRaw[nbAxis] = {0, 0, 0};

    // Get corrected data from gyro and accelero
    inertialMeasurementUnit.GetCorrectedAccelGyro(accRaw, gyroRaw);

    // Compute rotation speed using gyroscopes
    for (int axis = 0; axis < nbAxis; axis++)
        _angularSpeed[axis] = gyroRaw[axis];

    Normalize(accRaw, nbAxis);

    // Use complementary filter to merge gyro and accelerometer data
    // High pass filter on gyro, and low pass filter on accelerometer
    _angularPos[0] = HighPassFilterCoeff * (_angularPos[0] + (gyroRaw[0]) * _loop_time)
              + (1 - HighPassFilterCoeff) * RAD2DEG(atan(accRaw[1] / accRaw[2]));
    _angularPos[1] = HighPassFilterCoeff * (_angularPos[1] + (gyroRaw[1]) * _loop_time)
              + (1 - HighPassFilterCoeff) * RAD2DEG(-atan(accRaw[0] / accRaw[2]));
}

void Stabilization::PrintAccroModeParameters() {
    Serial.println(F("/********* PID settings *********/"));
    rollSpeedPID_Accro.PrintGains();
    pitchSpeedPID_Accro.PrintGains();
    yawControlLoop.PrintGains();
    Serial.print(F("Mixing: "));
    Serial.println(mixing);
}

void Stabilization::PrintAngleModeParameters() {
    Serial.println(F("/********* PID settings *********/"));
    rollPosPID_Angle.PrintGains();
    pitchPosPID_Angle.PrintGains();

    rollSpeedPID_Angle.PrintGains();
    pitchSpeedPID_Angle.PrintGains();
    yawControlLoop.PrintGains();
    Serial.println(F("/********* Complementary filter *********/"));
    Serial.print("Coefficient: ");
    Serial.print(HighPassFilterCoeff);
    Serial.print(" Time constant: ");
    Serial.println(GetFilterTimeConstant(0.00249));
    Serial.print(F("Mixing: "));
    Serial.println(mixing);
}

void Stabilization::ResetPID() {
    pitchMotorPwr = rollMotorPwr = yawMotorPwr = 0; // No correction if throttle put to min
    rollPosPID_Angle.Reset();
    pitchPosPID_Angle.Reset();
    rollSpeedPID_Angle.Reset();
    pitchSpeedPID_Angle.Reset();
    yawControlLoop.Reset();
    rollSpeedPID_Accro.Reset();
    pitchSpeedPID_Accro.Reset();

    SetMotorsPwrXConfig();
}

//    X configuration:
//  Motor0(CCW)  Motor1
//         \  /
//         /  \
//   Motor3   Motor2(CCW)
//
void Stabilization::SetMotorsPwrXConfig() {
    int throttle = GetThrottle();
    motorsSpeedControl.write(Motor0, throttle - pitchMotorPwr * mixing + rollMotorPwr * mixing
                                             - yawMotorPwr * mixing);
    motorsSpeedControl.write(Motor1, throttle - pitchMotorPwr * mixing - rollMotorPwr * mixing
                                             + yawMotorPwr * mixing);
    motorsSpeedControl.write(Motor2, throttle + pitchMotorPwr * mixing - rollMotorPwr * mixing
                                             - yawMotorPwr * mixing);
    motorsSpeedControl.write(Motor3, throttle + pitchMotorPwr * mixing + rollMotorPwr * mixing
                                             + yawMotorPwr * mixing);
}

void Stabilization::Idle() {
    motorsSpeedControl.Idle();
}
