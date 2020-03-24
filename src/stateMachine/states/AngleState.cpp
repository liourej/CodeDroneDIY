#include "AngleState.h"
#include "AccroState.h"
#include "SafetyState.h"
#include "../../stabilization/Stabilization.h"

extern Stabilization stabilization;

void AngleState::Run(StateMachine *_stateMachine, const float _loopTimeSec) {
    if (!stabilization.IsThrottleIdle()) {
        _stateMachine->throttleWasHigh = true;
        stabilization.Angle(_loopTimeSec);

        // Allow to change flying mode during flight
        if (stabilization.GetFlyingMode() == accro) {
            Serial.println(F("Flying mode changed from angle to accro"));
            SetState(_stateMachine, AccroState::GetInstance());
        }
    } else {
        // after 20s without pwr
        stabilization.ResetPID();
        if (_stateMachine->IsSafetyStateNeeded()) // Safety cut mngt: set safety cut
            SetState(_stateMachine, SafetyState::GetInstance());
    }
    SetState(_stateMachine, this);
}
