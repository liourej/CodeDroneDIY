#include "StateMachine.h"

extern StateMachine stateMachine;
extern float loopTimeSec;

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
