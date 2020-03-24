#include "AccroState.h"
#include "AngleState.h"
#include "SafetyState.h"
#include "../../stabilization/Stabilization.h"

extern Stabilization stabilization;

void AccroState::Run(StateMachine *_stateMachine, const float _loopTimeSec) {
    if (!stabilization.IsThrottleIdle()) {
        _stateMachine->throttleWasHigh = true;

        stabilization.Accro(_loopTimeSec);

        // Allow to change flying mode during flight
        if (stabilization.GetFlyingMode() == angle) {
            Serial.println(F("Flying mode changed from accro to angle"));
            SetState(_stateMachine, AngleState::GetInstance());
        }
    } else {
        // after 5s without pwr
        stabilization.ResetPID();
        if (_stateMachine->IsSafetyStateNeeded()) // Safety cut mngt: set safety cut
            SetState(_stateMachine, SafetyState::GetInstance());
    }
    SetState(_stateMachine, this);
}
