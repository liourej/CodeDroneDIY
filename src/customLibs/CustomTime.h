#ifndef CUSTOMTIME_H_
#define CUSTOMTIME_H_

#include "CustomSerialPrint.h"

class CustomTime {
  private:
    float startTime = 0.0;
    float prev_time = 0.0;
    float loop_time = 0.0;
    bool first_loop = true;

  public:
    void Init();

    float GetloopTimeSeconds();
    float GetExecutionTimeSeconds();

    float GetloopTimeMilliseconds();
    float GetExecutionTimeMilliseconds();
    void ComputeMeanLoopTime(const float _loopTimeSec, float &_meanLoopTime, uint16_t &_loopNb);
};

#endif // CUSTOMTIME_H_
