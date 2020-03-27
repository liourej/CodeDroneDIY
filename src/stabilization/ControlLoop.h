#ifndef CONTROLLOOP_H_
#define CONTROLLOOP_H_

#include "Arduino.h"
#include "ControlLoopConstants.h"
#include "../customLibs/CustomSerialPrint.h"

class ControlLoop {
  private:
    Constants constants;

    float error = 0;
    float errorPrev = 0;
    float integrator = 0;

  public:
    void SetGains(Constants _constants);
    void Reset();
    int ComputeCorrection(float _cmd, float _pos, float _loopTime);
    void PrintGains(void);
};

#endif // CONTROLLOOP_H_
