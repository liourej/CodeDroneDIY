#ifndef TIME_H_
#define TIME_H_

#include <Arduino.h>

#define COUNTER_NB 2

class Time {
  private:
    float startTime[COUNTER_NB] = {0.0, 0.0};
    float prev_time[COUNTER_NB] = {0.0, 0.0};
    float loop_time[COUNTER_NB] = {0.0, 0.0}; // sec
    bool first_loop[COUNTER_NB] = {true, true};

  public:
    void InitAllCounters();
    void Init(int _counter);

    float GetloopTimeSeconds(int _counter);
    float GetExecutionTimeSeconds(int _counter);

    float GetloopTimeMilliseconds(int _counter);
    float GetExecutionTimeMilliseconds(int _counter);
};

#endif // TIME_H_
