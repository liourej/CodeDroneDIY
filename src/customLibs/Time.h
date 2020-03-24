#ifndef TIME_H_
#define TIME_H_

#include <Arduino.h>

class Time {
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

#endif // TIME_H_
