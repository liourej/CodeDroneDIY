#include "Arduino.h"
#include "MotorsSpeedControl.h"

void MotorsSpeedControl::Init() {
    attach(Motor0, 4);
    attach(Motor1, 5);
    attach(Motor2, 6);
    attach(Motor3, 7);
    Idle();
}

void MotorsSpeedControl::attach(int _id, int _pin) {
    motorsList[_id].pin = _pin;
    pinMode(_pin, OUTPUT);
} // set motor pin to output};

void MotorsSpeedControl::Idle() {
    for (int id = 0; id < nbMotors; id++) {
        motorsList[id].ticks = usToTicks(MIN_POWER);
        motorsList[id].PWM = MIN_POWER;
    }
}

void MotorsSpeedControl::write(int _id, float _PWM) {
    if (_PWM < MIN_POWER) {
        motorsList[_id].PWM = MIN_POWER;
    } else if (_PWM > MAX_POWER) { // Check max power
        CustomSerialPrint::println(F("WARNING, MAX POWER REACHED!!"));
        motorsList[_id].PWM = MAX_POWER;
    } else {
        motorsList[_id].PWM = _PWM;
    }

    motorsList[_id].ticks = usToTicks(_PWM);
}

void MotorsSpeedControl::SetMotorsSpeed(volatile uint16_t *TCNTn, volatile uint16_t *OCRnA) {
    static bool firstLoop = true;
    if (firstLoop) {
        currMotor = 0;
        PORTD = B00010000;
        firstLoop = false;
    }

    if (currMotor == Motor0) {
        PORTD ^= B00110000; // Reset pin PD4 and set pin PD5 using XOR
        *OCRnA = motorsList[currMotor + 1].ticks;
    } else if (currMotor == Motor1) {
        PORTD ^= B01100000; // Reset pin PD5 and set pin PD6 using XOR
        *OCRnA = motorsList[currMotor + 1].ticks;
    } else if (currMotor == Motor2) {
        PORTD ^= B11000000; // Reset pin PD6 and set pin 7 using XOR
        *OCRnA = motorsList[currMotor + 1].ticks;
    } else {
        PORTD ^= B10010000; // Reset pin PD7 and set pin PD4 using XOR
        currMotor = -1;
        *OCRnA = motorsList[currMotor + 1].ticks;
    }
    *TCNTn = 0; // Reset timer
    currMotor++;
}
