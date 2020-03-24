#include <avr/wdt.h>
#include "Stabilization.h"

void Stabilization::Init() {
    // ESC
    ESCs.Init();

    // MPU6050: join I2C bus
    Wire.begin();
    Wire.setClock(400000L); // Communication with MPU-6050 at 400KHz

    while (!Rx.IsReady()) {
        Serial.println(F("Rx not ready, try again, please wait. "));
        ESCs.Idle();
        wdt_reset();
        delay(200);
    }
    // MPU6050, MS5611: initialize MPU6050 and MS5611 devices (IMU and
    // barometer)

    attitude.Init();

    if ((ESCs.GetESCsMaxPower() == 1860) && (ESCs.GetESCsMaxThrottle() >= (1860 * 0.8)))
        Serial.println(
                F("!!!!!!!!!!!!!!!!!!!!FLYING MODE "
                  "POWER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! "));
    else if ((ESCs.GetESCsMaxPower() <= 1300))
        Serial.println(F("DEBUG MODE POWER!!! "));
    else
        Serial.println(F("UNEXPECTED POWER "));

    Serial.print(F("MAX_POWER: "));
    Serial.print(ESCs.GetESCsMaxPower());
    Serial.print(F(" MAX_THROTTLE_PERCENT: "));
    Serial.println(ESCs.GetESCsMaxThrottlePercent());
    // Angle mode PID config

    rollPosPID_Angle.SetGains(PIDConstants::GetInstance()->anglePosPIDParams);
    pitchPosPID_Angle.SetGains(PIDConstants::GetInstance()->anglePosPIDParams);
    rollSpeedPID_Angle.SetGains(PIDConstants::GetInstance()->angleSpeedPIDParams);
    pitchSpeedPID_Angle.SetGains(PIDConstants::GetInstance()->angleSpeedPIDParams);

    // Adjust Kp from potentiometer on A0
    PIDConstants::GetInstance()->yawSpeedPIDParams[1] = map(analogRead(0), 0, 1023, 0, 500);
    Serial.print("Yaw kP: ");
    Serial.println(PIDConstants::GetInstance()->yawSpeedPIDParams[1]);
    yawSpeedPID_Angle.SetGains(PIDConstants::GetInstance()->yawSpeedPIDParams);

    // Accro mode PID config
    rollSpeedPID_Accro.SetGains(PIDConstants::GetInstance()->accroSpeedPIDParams);
    pitchSpeedPID_Accro.SetGains(PIDConstants::GetInstance()->accroSpeedPIDParams);
    yawSpeedPID_Accro.SetGains(PIDConstants::GetInstance()->yawSpeedPIDParams);
}

void Stabilization::Accro(float _loopTimeSec) {
    // Get current attitude (roll, pitch, yaw speeds)
    attitude.GetCurrPos(posCurr, speedCurr, _loopTimeSec);

    // Compute new speed commandi for each axis
    rollMotorPwr =
            rollSpeedPID_Accro.ComputeCorrection(Rx.GetAileronsSpeed(), speedCurr[0], _loopTimeSec);
    pitchMotorPwr = pitchSpeedPID_Accro.ComputeCorrection(Rx.GetElevatorSpeed(), speedCurr[1],
                                                          _loopTimeSec);
    yawMotorPwr = yawSpeedPID_Accro.ComputeCorrection(Rx.GetRudder(), speedCurr[2], _loopTimeSec);

    // Apply computed speed command to motors
    SetMotorsPwrXConfig();
}

void Stabilization::Angle(float _loopTimeSec) {
    // Get current attitude (roll, pitch, yaw angles and speeds)
    attitude.GetCurrPos(posCurr, speedCurr, _loopTimeSec);

    // Compute roll position command
    rollPosCmd =
            rollPosPID_Angle.ComputeCorrection(Rx.GetAileronsAngle(), posCurr[0], _loopTimeSec);

    // Compute roll speed command
    rollMotorPwr = rollSpeedPID_Angle.ComputeCorrection(rollPosCmd, speedCurr[0], _loopTimeSec);

    // Compute pitch position command
    pitchPosCmd =
            pitchPosPID_Angle.ComputeCorrection(Rx.GetElevatorAngle(), posCurr[1], _loopTimeSec);

    // Compute pitch speed command
    pitchMotorPwr = pitchSpeedPID_Angle.ComputeCorrection(pitchPosCmd, speedCurr[1], _loopTimeSec);

    // Compute yaw speed command
    yawMotorPwr = yawSpeedPID_Angle.ComputeCorrection(Rx.GetRudder(), speedCurr[2], _loopTimeSec);

    // Apply computed command to motors
    SetMotorsPwrXConfig();
}

void Stabilization::PrintAccroModeParameters() {
    Serial.println(F("/********* PID settings *********/"));
    rollSpeedPID_Accro.PrintGains();
    pitchSpeedPID_Accro.PrintGains();
    yawSpeedPID_Accro.PrintGains();
    Serial.print(F("Mixing: "));
    Serial.println(mixing);
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
    Serial.print(attitude.HighPassFilterCoeff);
    Serial.print(" Time constant: ");
    Serial.println(attitude.GetFilterTimeConstant(0.00249));
    Serial.print(F("Mixing: "));
    Serial.println(mixing);
}

void Stabilization::ResetPID() {
    pitchMotorPwr = rollMotorPwr = yawMotorPwr = 0; // No correction if throttle put to min
    rollPosPID_Angle.Reset();
    pitchPosPID_Angle.Reset();
    rollSpeedPID_Angle.Reset();
    pitchSpeedPID_Angle.Reset();
    yawSpeedPID_Angle.Reset();
    rollSpeedPID_Accro.Reset();
    pitchSpeedPID_Accro.Reset();
    yawSpeedPID_Accro.Reset();

    SetMotorsPwrXConfig();
}

//    X configuration:
//  ESC0(CCW)  ESC1
//         \  /
//         /  \
//     ESC3   ESC2(CCW)
//
void Stabilization::SetMotorsPwrXConfig() {
    int throttle = GetThrottle();
    ESCs.write(ESC0,
               throttle - pitchMotorPwr * mixing + rollMotorPwr * mixing - yawMotorPwr * mixing);
    ESCs.write(ESC1,
               throttle - pitchMotorPwr * mixing - rollMotorPwr * mixing + yawMotorPwr * mixing);
    ESCs.write(ESC2,
               throttle + pitchMotorPwr * mixing - rollMotorPwr * mixing - yawMotorPwr * mixing);
    ESCs.write(ESC3,
               throttle + pitchMotorPwr * mixing + rollMotorPwr * mixing + yawMotorPwr * mixing);
}

void Stabilization::Idle() {
    ESCs.Idle();
}
