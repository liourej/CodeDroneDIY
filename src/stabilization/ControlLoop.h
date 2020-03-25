#ifndef CONTROLLOOP_H_
#define CONTROLLOOP_H_

#include "Arduino.h"

class ControlLoop {
  private:
    float G = 0;
    float Kp, Kd, Ki = 0;

    float error = 0;
    float errorPrev = 0;
    float integrator = 0;

  public:
    void SetGains(const float _params[4]);
    void Reset();
    int ComputeCorrection(float _cmd, float _pos, float _loopTime);
    void PrintGains(void);
};

#endif // CONTROLLOOP_H_
