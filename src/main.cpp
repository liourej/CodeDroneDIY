#include <avr/wdt.h>
#include <math.h>

#include "Time.h"
#include "Reception.h"
#include "Stabilization.h"
#include "StateMachine.h"

Time time;
Reception Rx;
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
    Rx.GetWidth();
}

void PrintSettings() {
    Serial.println(F("/********* settings *********/"));
    Serial.println(F("FLYING_MODE_ANGLE"));
    stabilization.PrintAngleModeParameters();
    Serial.println(F("FLYING_MODE_ACCRO"));
    stabilization.PrintAccroModeParameters();

    Serial.println(F("/********* Receiver settings *********/"));
    Rx.PrintCmd();
    Serial.println(F("/********* MPU 6050 Configuration *********/"));
}

// Initialiaze all sensors and communication pipes
void setup() {
    InitTimer1();

    // Receiver
    attachInterrupt(0, RxInterrupt, RISING); // Receiver interrupt on PD2 (INT0)

    // Console print: initialize serial communication
    Serial.begin(230400);

    stabilization.Init(Rx);

    time.InitAllCounters();
    stateMachine.Init();

    // Set watchdog reset
    wdt_enable(WDTO_1S);

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

float loopTimeSec = 0;
typedef void *(*StateFunc)();

// Main loop
void loop() {
    StateFunc statefunc;
    static uint8_t loopNb = 0;
    static float meanLoopTime = 0;
    uint8_t throttle = 0;
    loopTimeSec = time.GetloopTimeMilliseconds(0);

    // State Machine initialization -> starting -> angle/accro -> safety -> disarmed -> angle/accro
    statefunc = (StateFunc)(*statefunc)();

    // Compute mean loop time and complementary filter time constant
    int state = Rx.GetFlyingMode();
    if (((state == angle) || (state == accro))
        && (throttle > stabilization.GetESCIdleThreshold())) {
        if (loopNb > 1000) {
            meanLoopTime = meanLoopTime / loopNb;
            Serial.println(meanLoopTime, 2);
            meanLoopTime = 0;
            loopNb = 0;
        } else {
            meanLoopTime += loopTimeSec;
            loopNb++;
        }
    }
    wdt_reset();
}
