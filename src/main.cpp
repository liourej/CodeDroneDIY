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
void PrintConfig();

// Initialiaze all sensors and communication pipes
void setup() {
    CustomSerialPrint::begin(230400); // Console print: initialize serial communication

    stabilization.Init();
    time.Init();
    stateMachine.Init();

    wdt_enable(WDTO_1S); // Set watchdog reset
    PrintConfig();
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
            time.ComputeMeanLoopTime(loopTimeSec, meanLoopTime, loopNb);
        }
    }
    wdt_reset();
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
#endif