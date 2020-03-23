#include "StateMachine.h"

extern StateMachine stateMachine;
extern float loopTimeSec;

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
