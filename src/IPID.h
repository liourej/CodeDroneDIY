#ifndef IPID_H_
#define IPID_H_

#include "Arduino.h"

class IPID {
  public:
    void Reset();
    int ComputeCorrection(float _cmd, float _pos, float _loopTime);
    void PrintGains(void);
};

#endif // IPID_H_
