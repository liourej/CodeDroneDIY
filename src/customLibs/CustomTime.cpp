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

void CustomTime::ComputeMeanLoopTime(const float _loopTimeSec, float &_meanLoopTime, uint16_t &_loopNb) {
    if (_loopNb > 1000) {
        _meanLoopTime = _meanLoopTime / _loopNb;
        CustomSerialPrint::println(_meanLoopTime, 2);
        _meanLoopTime = 0;
        _loopNb = 0;
    } else {
        _meanLoopTime += _loopTimeSec;
        _loopNb++;
    }
}