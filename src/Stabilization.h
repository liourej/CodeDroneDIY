#ifndef STABILIZATION_H_
#define STABILIZATION_H_

#include "ESC.h"
#include "Time.h"
#include "Reception.h"
#include "StateMachine.h"
#include "Attitude.h"
#include "PID.h"

class Stabilization {
  private:
    // PID setup
    const float GAIN = 0.010;

    // Angle mode
    float anglePosPIDParams[4] = {0.010, 268, 0.5, 0.0};  // G, Kp, Kd, Ki
    float angleSpeedPIDParams[4] = {0.010, 192, 0.0, 0.0};
    float altiSpeedPIDParams[4] = {0.010, 10, 0.0, 0.0};

    // Accro mode
    // 450mm frame, 10x4.5" bi-pale - Tested during flight test: OK
    float accroSpeedPIDParams[4] = { 0.010, 192, 0.0, 0.0};

    float ACCRO_YAW_KP = 0;  // Not used for now

    // Yaw PID
    float yawSpeedPIDParams[4] = { 0.010, 150.0, 0.0, 0.0};  // G, Kp, Kd, Ki

    int rollPosCmd, pitchPosCmd = 0;
    StateMachine stateMachine;
    int rollMotorPwr, pitchMotorPwr, yawMotorPwr = 0;
    float speedCurr[3] = { 0.0, 0.0, 0.0 };  // Teta speed (°/s) (only use gyro)
    float posCurr[3] = { 0.0, 0.0, 0.0 };  // Teta position (°) (use gyro + accelero)
    Reception Rx;
    uint8_t throttle = 0;
    ESC ESCs;
    PID rollPosPID_Angle, pitchPosPID_Angle, yawPosPID_Angle;
    PID rollSpeedPID_Angle, pitchSpeedPID_Angle, yawSpeedPID_Angle;
    PID rollSpeedPID_Accro, pitchSpeedPID_Accro, yawSpeedPID_Accro;
    Attitude Attitude;
  public:
    void Init();
    void Accro(float _loopTimeSec, Reception &_Rx);
    void Angle(float _loopTimeSec, Reception &_Rx);
    void PrintAccroModeParameters();
    void PrintAngleModeParameters();
    void ResetPID(int *_rollMotorPwr, int *_pitchMotorPwr, int *_yawMotorPwr);
};
#endif // STABILIZATION_H_
