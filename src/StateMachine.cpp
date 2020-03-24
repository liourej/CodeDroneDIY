#include "StateMachine.h"

void StateMachine::Init() {
    // Buzzer
    pinMode(BUZZER_PIN, OUTPUT);

    elapsedTime.Init();
    timeBuzzer.Init();
}

// Activate buzzer after x minutes of power idle
void StateMachine::ActivateBuzzer(int _durationSec) {
    if (timeBuzzer.GetExecutionTimeSeconds() > _durationSec) {
        while (true) {
            digitalWrite(BUZZER_PIN, HIGH);
            delayMicroseconds(1800); // Dirty, it would be better to use timer interrupt
            digitalWrite(BUZZER_PIN, LOW);
            delay(10);
            wdt_reset();
            Serial.println(F("BUZZZZZ"));
        }
    }
}

// Auto disarm when throttle is idle since a long period
bool StateMachine::IsSafetyStateNeeded() {
    if (throttleWasHigh) {
        Serial.println(F("Throttle just setted low!"));
        Init();
        throttleWasHigh = false;
    } else if (elapsedTime.GetExecutionTimeSeconds() > delayThresholdSec) {
        Serial.print(delayThresholdSec);
        Serial.println(F(" sec without power, system DISARMED!"));
        return true;
    }
    return false;
}
