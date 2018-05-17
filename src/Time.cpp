#include <Arduino.h>
#include "Time.h"

void Time::InitAllCounters() {
    for (int i = 0; i < COUNTER_NB; i++) {
        startTime[COUNTER_NB] = millis();
        prev_time[COUNTER_NB] = millis();
    }
}

void Time::Init(int _counter) {
    if (_counter > COUNTER_NB)
        return;

    startTime[_counter] = millis();
    prev_time[_counter] = millis();
}

float Time::GetloopTimeSeconds(int _counter) {
    return (GetloopTimeMilliseconds(_counter) / 1000);
}

float Time::GetExecutionTimeSeconds(int _counter) {
    return (GetExecutionTimeMilliseconds(_counter) / 1000);
}

float Time::GetloopTimeMilliseconds(int _counter) {
    if (_counter > COUNTER_NB)
        return -1;

    loop_time[_counter] = (millis() - prev_time[_counter]);
    prev_time[_counter] = millis();

    return loop_time[_counter];
}

float Time::GetExecutionTimeMilliseconds(int _counter) {
    if (_counter > COUNTER_NB)
        return -1;

    return (millis() - startTime[_counter]);
}
