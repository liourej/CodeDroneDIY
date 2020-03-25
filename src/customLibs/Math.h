#ifndef MATH_H_
#define MATH_H_

#include <avr/wdt.h>
#include <limits.h>
#include <math.h>

#define RAD2DEG(angle) angle * 180 / PI

class Math {
  protected:
    bool ComputeDelta(int16_t _list[], int _size, int16_t *_delta);
    bool ComputeMean(int16_t _list[], int _size, int16_t _deltaThreshold, float *_mean);
    inline void Normalize(float _vectorIn[], const int vectorSize);
};

#endif // MATH_H_
