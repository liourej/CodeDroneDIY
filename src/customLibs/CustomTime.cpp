#include <Arduino.h>
#include "CustomTime.h"

void CustomTime::Init() {
    startTime = millis();
    prev_time = millis();
}

float CustomTime::GetloopTimeSeconds() {
    return GetloopTimeMilliseconds() / 1000;
}

float CustomTime::GetExecutionTimeSeconds() {
    return GetExecutionTimeMilliseconds() / 1000;
}

float CustomTime::GetloopTimeMilliseconds() {
    loop_time = millis() - prev_time;
    prev_time = millis();

    return loop_time;
}

float CustomTime::GetExecutionTimeMilliseconds() {
    return millis() - startTime;
}
