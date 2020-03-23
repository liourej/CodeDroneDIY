#include "StateMachine.h"

extern StateMachine stateMachine;
extern float loopTimeSec;

// States
void *initState(const float) {
    stabilization.Idle();
    while (!stabilization.AreAttitudeOffsetsComputed())
        stabilization.AttitudeComputeOffsets();

    if (stabilization.GetFlyingMode() != disarmed)
        return (void *)&initState;
    else if (stabilization.AreAttitudeOffsetsComputed())
        return (void *)&startingState;

    return (void *)&initState;
}
void *startingState(const float) {
    stabilization.Idle();
    int state = stabilization.GetFlyingMode();
    delay(500);
    if (state != stabilization.GetFlyingMode()) // Check it was not a transitory switch state
        return (void *)&startingState;
    if ((state == angle) || (state == accro)) {
        Serial.println(F("stateMachine.state != disarmed MODE"));
        // PrintSettings();
        if (state == angle)
            return (void *)&angleState;
        if (state == accro)
            return (void *)&accroState;
    }
    return (void *)&startingState;
}

void *angleState(const float _loopTimeSec) {
    if (!stabilization.IsThrottleIdle()) {
        stateMachine.throttleWasHigh = true;
        stabilization.Angle(_loopTimeSec);

        // Allow to change flying mode during flight
        if (stabilization.GetFlyingMode() == accro) {
            Serial.println(F("Flying mode changed from angle to accro"));
            return (void *)&accroState;
        }
    } else {
        // after 20s without pwr
        stabilization.ResetPID();
        if (stateMachine.IsSafetyStateNeeded()) // Safety cut mngt: set safety cut
            return (void *)&safetyState;
    }
    return (void *)&angleState;
}

void *accroState(const float _loopTimeSec) {
    if (!stabilization.IsThrottleIdle()) {
        stateMachine.throttleWasHigh = true;

        stabilization.Accro(_loopTimeSec);

        // Allow to change flying mode during flight
        if (stabilization.GetFlyingMode() == angle) {
            Serial.println(F("Flying mode changed from accro to angle"));
            return (void *)&angleState;
        }
    } else {
        // after 5s without pwr
        stabilization.ResetPID();
        if (stateMachine.IsSafetyStateNeeded()) // Safety cut mngt: set safety cut
            return (void *)&safetyState;
    }
    return (void *)&accroState;
}

void *safetyState(const float) {
    stabilization.Idle();
    stateMachine.ActivateBuzzer(120);
    stabilization.GetFlyingMode();
    if (stabilization.GetFlyingMode() != disarmed) {
        stabilization.Idle();
        return (void *)&safetyState;
    } else {
        return (void *)&disarmedState;
    }
}

void *disarmedState(const float) {
    stabilization.Idle();
    stateMachine.ActivateBuzzer(120);
    int state = stabilization.GetFlyingMode();
    delay(500);
    // Check it was not a transitory switch state
    if (state != stabilization.GetFlyingMode())
        return (void *)&disarmedState;
    if (state != disarmed) {
        stateMachine.throttleWasHigh = true;
        if (state == angle) {
            Serial.println(F("ANGLE MODE"));
            return (void *)&angleState;
        } else if (state == accro) {
            Serial.println(F("ACCRO MODE"));
            return (void *)&accroState;
        }
    }
    return (void *)&disarmedState;
}

void StateMachine::Init() {
    // Buzzer
    pinMode(BUZZER_PIN, OUTPUT);

    elapsedTime.Init(0);
    timeBuzzer.Init(0);
}

// Activate buzzer after x minutes of power idle
void StateMachine::ActivateBuzzer(int _durationSec) {
    if (timeBuzzer.GetExecutionTimeSeconds(0) > _durationSec) {
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
    } else if (elapsedTime.GetExecutionTimeSeconds(0) > delayThresholdSec) {
        Serial.print(delayThresholdSec);
        Serial.println(F(" sec without power, system DISARMED!"));
        return true;
    }
    return false;
}
