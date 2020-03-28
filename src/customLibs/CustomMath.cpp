#include "CustomMath.h"

bool CustomMath::ComputeDelta(int16_t _list[], int _size, int16_t *_delta) {

    if (_size <= 2)
        return false;

    int16_t maxVal = _list[0];
    int16_t minVal = _list[0];

    CustomSerialPrint::println(F("ComputeDelta : Samples:"));
    for (int sample = 0; sample < _size; sample++) {
        CustomSerialPrint::println(_list[sample]);
        if (_list[sample] > maxVal)
            maxVal = _list[sample];
        if (_list[sample] < minVal)
            minVal = _list[sample];
    }
    (*_delta) = abs(maxVal - minVal);
    CustomSerialPrint::println(F("ComputeDelta: delta:"));
    CustomSerialPrint::println(*_delta);
    return true;
}

bool CustomMath::ComputeMean(int16_t _list[], int _size, int16_t _deltaThreshold, float *_mean) {
    if (_size <= 0)
        return false;

    // Check delta before computing mean
    int16_t delta = 0;
    if (!ComputeDelta(_list, _size, &delta)) {
        CustomSerialPrint::println(F("ComputeMean: Error, ComputeDelta failed"));
        return false;
    }
    if (delta > _deltaThreshold) {
        CustomSerialPrint::println(F("ComputeMean: Excessive delta:"));
        CustomSerialPrint::println(F("delta:"));
        CustomSerialPrint::println(delta);
        CustomSerialPrint::println(F("Threshold:"));
        CustomSerialPrint::println(_deltaThreshold);
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

void CustomMath::VectorNormalize(float _vectorIn[], const int vectorSize) {
    float sumSquares = 0.0;
    for (int index = 0; index < vectorSize; index++)
        sumSquares += _vectorIn[index] * _vectorIn[index];

    float norm = sqrt(sumSquares);

    if (norm > 0.0)
        for (int index = 0; index < vectorSize; index++)
            _vectorIn[index] = _vectorIn[index] / norm;
    else
        for (int index = 0; index < vectorSize; index++)
            _vectorIn[index] = 0.0;
}
