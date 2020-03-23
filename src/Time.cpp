#include <Arduino.h>
#include "Time.h"

void Time::Init() {
    startTime = millis();
    prev_time = millis();
}

float Time::GetloopTimeSeconds() {
    return GetloopTimeMilliseconds() / 1000;
}

float Time::GetExecutionTimeSeconds() {
    return GetExecutionTimeMilliseconds() / 1000;
}

float Time::GetloopTimeMilliseconds() {
    loop_time = millis() - prev_time;
    prev_time = millis();

    return loop_time;
}

float Time::GetExecutionTimeMilliseconds() {
    return millis() - startTime;
}
