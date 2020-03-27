#ifndef UNIT_TEST
#include <avr/wdt.h>
#include <math.h>

#include "customLibs/CustomTime.h"
#include "stabilization/hardware/RadioReception.h"

#include "stabilization/Stabilization.h"
#include "stateMachine/StateMachine.h"

CustomTime time;
Stabilization stabilization;
StateMachine stateMachine;

// Timer interrupt to set PWM to motors controllers
typedef enum { _timer1, _Nbr_16timers } timer16_Sequence_t;

static inline void handle_interrupts(timer16_Sequence_t timer, volatile uint16_t *TCNTn,
                                     volatile uint16_t *OCRnA) {
    stabilization.SetMotorsSpeed(TCNTn, OCRnA);
}

SIGNAL(TIMER1_COMPA_vect) {
    handle_interrupts(_timer1, &TCNT1, &OCR1A);
}

void InitTimer1() {
    TCCR1A = 0;         // normal counting mode
    TCCR1B = _BV(CS10); // no prescaler
    TCNT1 = 0;          // clear the timer count
    OCR1A = usToTicks(stabilization.GetMotorsMinPower());

    TIFR1 |= _BV(OCF1A);   // clear any pending interrupts;
    TIMSK1 |= _BV(OCIE1A); // enable the output compare interrupt
}

// Interrupt to decode cppm signal received from RC transmitter
void RxInterrupt() {
    stabilization.ComputeRxImpulsionWidth();
}

void PrintConfig() {
    if ((stabilization.GetMotorsMaxPower() == 1860)
        && (stabilization.GetMotorsMaxThrottle() >= (1860 * 0.8)))
        CustomSerialPrint::println(
                F("!!!!!!!!!!!!!!!!!!!!FLYING MODE "
                  "POWER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! "));
    else if ((stabilization.GetMotorsMaxPower() <= 1300))
        CustomSerialPrint::println(F("DEBUG MODE POWER!!! "));
    else
        CustomSerialPrint::println(F("UNEXPECTED POWER "));

    CustomSerialPrint::print(F("MAX_POWER: "));
    CustomSerialPrint::print(stabilization.GetMotorsMaxPower());
    CustomSerialPrint::print(F(" MAX_THROTTLE_PERCENT: "));
    // CustomSerialPrint::println(stabilization.GetMotorsMaxThrottlePercent());

    CustomSerialPrint::println(F("Setup Finished"));
}

// Initialiaze all sensors and communication pipes
void setup() {

    InitTimer1();

    attachInterrupt(0, RxInterrupt, RISING); // Receiver interrupt on PD2 (INT0)

    CustomSerialPrint::begin(230400); // Console print: initialize serial communication

    stabilization.Init();

    time.Init();

    stateMachine.Init();

    wdt_enable(WDTO_1S); // Set watchdog reset

    PrintConfig();
}

void ComputeMeanLoopTime(const float _loopTimeSec, float &_meanLoopTime, uint16_t &_loopNb) {
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

// Main loop
void loop() {
    float loopTimeSec = 0.0;
    uint16_t loopNb = 0;
    float meanLoopTime = 0.0;

    loopTimeSec = time.GetloopTimeMilliseconds();

    // State Machine initialization -> starting -> angle/accro -> safety -> disarmed -> angle/accro
    stateMachine.Run(loopTimeSec);

    // Compute mean loop time and complementary filter time constant
    int flyingMode = stabilization.GetFlyingMode();
    if ((flyingMode == angle) || (flyingMode == accro)) {
        if (!stabilization.IsThrottleIdle()) {
            ComputeMeanLoopTime(loopTimeSec, meanLoopTime, loopNb);
        }
    }
    wdt_reset();
}
#endif