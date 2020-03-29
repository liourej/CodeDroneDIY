#include "MotorsSpeedControl.h"

int MotorsSpeedControl::currMotor = -1;
uint16_t MotorsSpeedControl::motorsTicks[nbMotors] = {0, 0, 0, 0};

void MotorsSpeedControl::Init() {
    // Set motors pin PD4, PD5, PD6, PD7 as outputs
    DDRD |= (1 << DDD7)| (1 << DDD6)| (1 << DDD5)| (1 << DDD4);
    InitTimer1();
    Idle();
}

// Timer interrupt to set PWM to motors controllers
typedef enum { _timer1, _Nbr_16timers } timer16_Sequence_t;

static inline void handle_interrupts(timer16_Sequence_t timer, volatile uint16_t *TCNTn,
                                     volatile uint16_t *OCRnA) {
    MotorsSpeedControl::ApplySpeed(TCNTn, OCRnA);
}

// Timer 1 is use to drive motors using PWM
SIGNAL(TIMER1_COMPA_vect) {
    handle_interrupts(_timer1, &TCNT1, &OCR1A);
}

// Timer 1 is use to drive motors using PWM
void MotorsSpeedControl::InitTimer1() {
    TCCR1A = 0;         // normal counting mode
    TCCR1B = _BV(CS10); // no prescaler
    TCNT1 = 0;          // clear the timer count
    OCR1A = usToTicks(MIN_POWER);

    TIFR1 |= _BV(OCF1A);   // clear any pending interrupts;
    TIMSK1 |= _BV(OCIE1A); // enable the output compare interrupt
}

void MotorsSpeedControl::Idle() {
    for (int id = 0; id < nbMotors; id++)
        motorsTicks[id] = usToTicks(MIN_POWER);
}

void MotorsSpeedControl::UpdateSpeed(int _id, float _PWM) {
    if (_PWM < MIN_POWER) {
        motorsTicks[_id] = usToTicks(MIN_POWER);
    } else if (_PWM > MAX_POWER) { // Check max power
        CustomSerialPrint::println(F("WARNING, MAX POWER REACHED!!"));
        motorsTicks[_id] = usToTicks(MAX_POWER);
    } else {
        motorsTicks[_id] = usToTicks(_PWM);
    }
}

// Set a falling edge for the previous motor, and a rising edge for the current motor
// Motor speed is managed by the pulse with: the larger the high level is, the faster the motor run
void MotorsSpeedControl::ApplySpeed(volatile uint16_t *TCNTn, volatile uint16_t *OCRnA) {
    static bool firstLoop = true;
    if (firstLoop) {
        currMotor = 0;
        PORTD = B00010000;
        firstLoop = false;
    }

    if (currMotor == Motor0) { // Rising edge for Motor0, and falling edge for Motor3
        PORTD ^= B00110000; // Reset pin PD4 and set pin PD5 using XOR
        *OCRnA = motorsTicks[currMotor + 1]; // When TCNTn matches OCRnA, an output compare interrupt is thrown.
    } else if (currMotor == Motor1) {// Rising edge for Motor1, and falling edge for Motor0
        PORTD ^= B01100000; // Reset pin PD5 and set pin PD6 using XOR
        *OCRnA = motorsTicks[currMotor + 1];// When TCNTn matches OCRnA, an output compare interrupt is thrown.
    } else if (currMotor == Motor2) {// Rising edge for Motor2, and falling edge for Motor1
        PORTD ^= B11000000; // Reset pin PD6 and set pin 7 using XOR
        *OCRnA = motorsTicks[currMotor + 1];// When TCNTn matches OCRnA, an output compare interrupt is thrown.
    } else {// Rising edge for Motor3, and falling edge for Motor2
        PORTD ^= B10010000; // Reset pin PD7 and set pin PD4 using XOR
        currMotor = -1;
        *OCRnA = motorsTicks[currMotor + 1];// When TCNTn matches OCRnA, an output compare interrupt is thrown.
    }
    *TCNTn = 0; // Reset timer counter
    currMotor++;
}
