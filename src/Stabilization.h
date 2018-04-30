#ifndef STABILIZATION_H_
#define STABILIZATION_H_

#include "Reception.h"
#include "ESC.h"
#include "Attitude.h"
#include "PID.h"

class Stabilization {
  private:
    const float mixing = 0.5;

    // PID setup
    const float GAIN = 0.010;

    // Angle mode
    // These parameters are very important for flight success
    // They must be tuned for each frame type, motors, and propeller used
    float anglePosPIDParams[4] = {0.010, 268, 0.5, 0.0};  // G, Kp, Kd, Ki
    float angleSpeedPIDParams[4] = {0.010, 192, 0.0, 0.0};

    // Accro mode
    // 450mm frame, 10x4.5" 2 blades propellers - Tested during flight test: OK
    float accroSpeedPIDParams[4] = { 0.010, 192, 0.0, 0.0};

    // Yaw PID
    float yawSpeedPIDParams[4] = { 0.010, 150.0, 0.0, 0.0};  // G, Kp, Kd, Ki

    int rollPosCmd, pitchPosCmd = 0;
    int rollMotorPwr, pitchMotorPwr, yawMotorPwr = 0;
    float speedCurr[3] = { 0.0, 0.0, 0.0 };  // Teta speed (°/s) (only use gyro)
    float posCurr[3] = { 0.0, 0.0, 0.0 };  // Teta position (°) (use gyro + accelero)
    uint8_t throttle = 0;
    ESC ESCs;
    PID rollPosPID_Angle, pitchPosPID_Angle, yawPosPID_Angle;
    PID rollSpeedPID_Angle, pitchSpeedPID_Angle, yawSpeedPID_Angle;
    PID rollSpeedPID_Accro, pitchSpeedPID_Accro, yawSpeedPID_Accro;
    Attitude attitude;
  public:
    void XConfig(const int _throttle);
    void Init(Reception &_Rx);
    void Idle();
    void Accro(float _loopTimeSec, Reception &_Rx);
    void Angle(float _loopTimeSec, Reception &_Rx);
    void PrintAccroModeParameters();
    void PrintAngleModeParameters();
    void ResetPID();
    void SetESCsPWM(volatile uint16_t *TCNTn, volatile uint16_t* OCRnA) {ESCs.SetPWM_f5(TCNTn, OCRnA);}
    int GetESCsMaxPower() { return ESCs.MAX_POWER;}
    int GetESCsMinPower() { return ESCs.MIN_POWER;}
    int GetESCsMaxThrottlePercent() { return ESCs.MAX_THROTTLE_PERCENT;}
    int GetESCsMaxThrottle() { return ESCs.MAX_THROTTLE;}
    int GetESCIdleThreshold() { return ESCs.IDLE_THRESHOLD;}
    bool AreAttitudeOffsetsComputed() { return attitude.AreOffsetComputed(); }
    void AttitudeComputeOffsets() { attitude.ComputeOffsets();}
};
#endif // STABILIZATION_H_
