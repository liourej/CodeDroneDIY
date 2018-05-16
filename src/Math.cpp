#include "Math.h"

bool Math::ComputeDelta(int16_t _list[], int _size, int16_t* _delta) {
  int16_t maxVal = 0;
  int16_t minVal = INT_MAX;

  if (_size <= 2)
    return -1;

  for (int sample = 0; sample < _size; sample++) {
    if (_list[sample] > maxVal)
      maxVal = _list[sample];
    if (_list[sample] < minVal)
      minVal = _list[sample];
  }
  (*_delta) = maxVal - minVal;

  return 0;
}

bool Math::ComputeMean(int16_t _list[], int _size, int16_t _deltaThreshold, float* _mean) {
  if (_size <= 0)
    return false;

  // Check delta before computing mean
  int16_t delta = 0;
  if (ComputeDelta(_list, _size, &delta) != 0)
      return false;
  if (delta > _deltaThreshold)
    return false;

  // Compute mean
  (*_mean) = 0;
  for (int sample = 0; sample < _size; sample++) {
    (*_mean) = (*_mean) + _list[sample];
  }
  (*_mean) = (*_mean)/_size;

  return true;
}
