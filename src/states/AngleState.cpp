#include "StateMachine.h"

extern StateMachine stateMachine;
extern float loopTimeSec;

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
