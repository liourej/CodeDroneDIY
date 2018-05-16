#ifndef MATH_H_
#define MATH_H_

#include <avr/wdt.h>
#include <limits.h>

class Math
{
  protected:
    bool ComputeDelta(int16_t _list[], int _size, int16_t* _delta);
    bool ComputeMean(int16_t _list[], int _size, int16_t _deltaThreshold, float* _mean);
};

#endif // MATH_H_
