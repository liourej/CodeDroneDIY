#ifndef PID_H_
#define PID_H_

#include "Arduino.h"

class PID
{
  private:
    float G = 0;
    float Kp, Kd, Ki = 0;
    float speedCmd = 0;

    float posErrorPrev = 0;
    float posIntegrator = 0;

    float error = 0;
    float errorPrev = 0;
    float integrator = 0;

  public:
    void SetGains(float _params[4]);
    void Reset();
    int ComputeCorrection(float _cmd, float _pos, float _loopTime);
    void PrintGains(void);
};

#endif // PID_H_
