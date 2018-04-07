#include "Arduino.h"
#include "ESC.h"

extern unsigned int MIN_POWER;
extern unsigned int MAX_POWER;

void ESC::Init() {
  attach(ESC0, 4);
  attach(ESC1, 5);
  attach(ESC2, 6);
  attach(ESC3, 7);
  Idle();
}

void ESC::attach(int _id, int _pin) {
  ESCList[_id].pin = _pin;
  pinMode(_pin, OUTPUT);
}  // set servo pin to output};

void ESC::Idle() {
  for (int id = 0; id < 4; id++) {
    ESCList[id].ticks = usToTicks(MIN_POWER);
    ESCList[id].PWM = MIN_POWER;
  }
}

void ESC::write(int _id, float _PWM) {
  if (_PWM < MIN_POWER) {
    ESCList[_id].PWM = MIN_POWER;
  } else if (_PWM > MAX_POWER) {  // Check max power
    Serial.println(F("WARNING, MAX POWER REACHED!!"));
    ESCList[_id].PWM = MAX_POWER;
  } else {
    ESCList[_id].PWM = _PWM;
  }

  ESCList[_id].ticks = usToTicks(_PWM);
}

void ESC::SetPWM_f5(volatile uint16_t *TCNTn, volatile uint16_t* OCRnA) {
  static bool firstLoop = true;
  if (firstLoop) {
    currESC = 0;
    PORTD = B00010000;
    firstLoop = false;
  }

  if (currESC == ESC0) {
    PORTD ^= B00110000;  // Reset pin PD4 and set pin PD5 using XOR
    *OCRnA = ESCList[currESC + 1].ticks;
  } else if (currESC == ESC1) {
    PORTD ^= B01100000;  // Reset pin PD5 and set pin PD6 using XOR
    *OCRnA = ESCList[currESC + 1].ticks;
  } else if (currESC == ESC2) {
    PORTD ^= B11000000;  // Reset pin PD6 and set pin 7 using XOR
    *OCRnA = ESCList[currESC + 1].ticks;
  } else {
    PORTD ^= B10010000;  // Reset pin PD7 and set pin PD4 using XOR
    currESC = -1;
    *OCRnA = ESCList[currESC + 1].ticks;
  }
  *TCNTn = 0;  // Reset timer
  currESC++;
}
