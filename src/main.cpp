#include <avr/wdt.h>
#include <math.h>

#include "customLibs/Time.h"
#include "stabilization/Reception.h"
#include "stabilization/Stabilization.h"
#include "stateMachine/StateMachine.h"

Time time;

Stabilization stabilization;
StateMachine stateMachine;

// Timer interrupt to set PWM to motors controllers
typedef enum { _timer1, _Nbr_16timers } timer16_Sequence_t;

static inline void handle_interrupts(timer16_Sequence_t timer, volatile uint16_t *TCNTn,
                                     volatile uint16_t *OCRnA) {
    stabilization.SetESCsPWM(TCNTn, OCRnA);
}

SIGNAL(TIMER1_COMPA_vect) {
    handle_interrupts(_timer1, &TCNT1, &OCR1A);
}

void InitTimer1() {
    // Timer
    TCCR1A = 0;         // normal counting mode
    TCCR1B = _BV(CS10); // no prescaler
    TCNT1 = 0;          // clear the timer count
    OCR1A = usToTicks(stabilization.GetESCsMinPower());

    TIFR1 |= _BV(OCF1A);   // clear any pending interrupts;
    TIMSK1 |= _BV(OCIE1A); // enable the output compare interrupt
}

// Interrupt to decode cppm signal received from RC transmitter
void RxInterrupt() {
    stabilization.ComputeRxImpulsionWidth();
}

void PrintConfig() {
    if ((stabilization.GetESCsMaxPower() == 1860)
        && (stabilization.GetESCsMaxThrottle() >= (1860 * 0.8)))
        Serial.println(
                F("!!!!!!!!!!!!!!!!!!!!FLYING MODE "
                  "POWER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! "));
    else if ((stabilization.GetESCsMaxPower() <= 1300))
        Serial.println(F("DEBUG MODE POWER!!! "));
    else
        Serial.println(F("UNEXPECTED POWER "));

    Serial.print(F("MAX_POWER: "));
    Serial.print(stabilization.GetESCsMaxPower());
    Serial.print(F(" MAX_THROTTLE_PERCENT: "));
    Serial.println(stabilization.GetESCsMaxThrottlePercent());

    Serial.println(F("Setup Finished"));
}

// Initialiaze all sensors and communication pipes
void setup() {

    InitTimer1();

    attachInterrupt(0, RxInterrupt, RISING); // Receiver interrupt on PD2 (INT0)

    Serial.begin(230400); // Console print: initialize serial communication

    stabilization.Init();

    time.Init();

    stateMachine.Init();

    wdt_enable(WDTO_1S); // Set watchdog reset

    PrintConfig();
}

void ComputeMeanLoopTime(const float _loopTimeSec, uint16_t &_loopNb) {
    float meanLoopTime = 0;
    if (_loopNb > 1000) {
        meanLoopTime = meanLoopTime / _loopNb;
        Serial.println(meanLoopTime, 2);
        meanLoopTime = 0;
        _loopNb = 0;
    } else {
        meanLoopTime += _loopTimeSec;
        _loopNb++;
    }
}

// Main loop
void loop() {
    float loopTimeSec = 0.0;
    uint16_t loopNb = 0;

    loopTimeSec = time.GetloopTimeMilliseconds();

    // State Machine initialization -> starting -> angle/accro -> safety -> disarmed -> angle/accro
    stateMachine.Run(loopTimeSec);

    // Compute mean loop time and complementary filter time constant
    int flyingMode = stabilization.GetFlyingMode();
    if ((flyingMode == angle) || (flyingMode == accro)) {
        if (!stabilization.IsThrottleIdle()) {
            ComputeMeanLoopTime(loopTimeSec, loopNb);
        }
    }
    wdt_reset();
}
