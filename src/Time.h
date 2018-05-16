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
    void InitAllCounters() {
        for (int i = 0; i < COUNTER_NB; i++) {
            startTime[COUNTER_NB] = millis();
            prev_time[COUNTER_NB] = millis();
        }
    }

    void Init(int _counter) {
        if (_counter > COUNTER_NB)
            return;

        startTime[_counter] = millis();
        prev_time[_counter] = millis();
    }

    inline float GetloopTimeSeconds(int _counter) {
        return (GetloopTimeMilliseconds(_counter) / 1000);
    }

    float GetExecutionTimeSeconds(int _counter) {
        return (GetExecutionTimeMilliseconds(_counter) / 1000);
    }

    inline float GetloopTimeMilliseconds(int _counter) {
        if (_counter > COUNTER_NB)
            return -1;

        loop_time[_counter] = (millis() - prev_time[_counter]);
        prev_time[_counter] = millis();

        return loop_time[_counter];
    }

    float GetExecutionTimeMilliseconds(int _counter) {
        if (_counter > COUNTER_NB)
            return -1;

        return (millis() - startTime[_counter]);
    }
};

#endif // TIME_H_
