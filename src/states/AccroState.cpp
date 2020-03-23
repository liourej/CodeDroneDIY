#include "StateMachine.h"

extern StateMachine stateMachine;
extern float loopTimeSec;

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
