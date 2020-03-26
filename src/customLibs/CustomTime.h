#ifndef CUSTOMTIME_H_
#define CUSTOMTIME_H_

#include <Arduino.h>

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
};

#endif // CUSTOMTIME_H_
