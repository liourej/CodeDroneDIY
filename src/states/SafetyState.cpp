#include "StateMachine.h"

extern StateMachine stateMachine;
extern float loopTimeSec;

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
