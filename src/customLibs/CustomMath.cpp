#include "CustomMath.h"
#include "../libraries/I2Cdev/I2Cdev.h"

bool CustomMath::ComputeDelta(int16_t _list[], int _size, int16_t *_delta) {
    int16_t maxVal = 0;
    int16_t minVal = INT_MAX;

    if (_size <= 2)
        return false;

    for (int sample = 0; sample < _size; sample++) {
        if (_list[sample] > maxVal)
            maxVal = _list[sample];
        if (_list[sample] < minVal)
            minVal = _list[sample];
    }
    (*_delta) = maxVal - minVal;

    return true;
}

bool CustomMath::ComputeMean(int16_t _list[], int _size, int16_t _deltaThreshold, float *_mean) {
    if (_size <= 0)
        return false;

    // Check delta before computing mean
    int16_t delta = 0;
    if (!ComputeDelta(_list, _size, &delta)) {
        Serial.println(F("ComputeMean: Error, ComputeDelta failed"));
        return false;
    }
    if (delta > _deltaThreshold) {
        Serial.println(F("ComputeMean: Excessive delta:"));
        Serial.println(F("delta:"));
        Serial.println(delta);
        Serial.println(F("Threshold:"));
        Serial.println(_deltaThreshold);
        return false;
    }

    // Compute mean
    (*_mean) = 0;
    for (int sample = 0; sample < _size; sample++) {
        (*_mean) = (*_mean) + _list[sample];
    }
    (*_mean) = (*_mean) / _size;

    return true;
}

inline void CustomMath::Normalize(float _vector[], const int vectorSize) {
    float sumSquares = 0.0;
    for (int index = 0; index < vectorSize; index++)
        sumSquares += _vector[index] * _vector[index];

    float norm = sqrt(sumSquares);

    for (int index = 0; index < vectorSize; index++)
        _vector[index] = _vector[index] / norm;
}