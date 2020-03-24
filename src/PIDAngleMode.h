#ifndef PIDAngleMode_H_
#define PIDAngleMode_H_

#include "Arduino.h"

class PIDAngleMode: public IPID {
  private:
    static const float GSpeed = 0.010;
    static const float KpSpeed = 192.0;
    static const float KdSpeed = 0.0;
    static const float KiSpeed = 0.0;

    static const float GPos= 0.010;
    static const float KpPos = 268.0;
    static const float KdPos = 0.5;
    static const float KiPos = 0.0;
  private :
int compute

  public :
    int ComputeCorrection(float _cmd, float _pos,float _speed, float _loopTimeSec)
    {
      int rollPosCmd =
              rollPosPID_Angle.ComputeCorrection(_cmd, _pos, _loopTimeSec);

      // Compute roll speed command
      rollMotorPwr = rollSpeedPID_Angle.ComputeCorrection(rollPosCmd, _speed, _loopTimeSec);
    }
};

#endif // PID_H_
