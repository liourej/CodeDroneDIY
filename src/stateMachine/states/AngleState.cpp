#include "AngleState.h"
#include "AccroState.h"
#include "SafetyState.h"
#ifndef UNIT_TEST
#include "../../stabilization/Stabilization.h"
#else
#include "../../stabilization/StabilizationStub.h"
#endif

#ifndef UNIT_TEST
extern Stabilization stabilization;
#else
extern StabilizationStub stabilization;
#endif

void AngleState::Run(StateMachine *_stateMachine, const float _loopTimeSec) {
    if (!stabilization.IsThrottleIdle()) {
        _stateMachine->throttleWasHigh = true;
        stabilization.Angle(_loopTimeSec);

        // Allow to change flying mode during flight
        if (stabilization.GetFlyingMode() == accro) {
            Serial.println(F("Flying mode changed from angle to accro"));
            SetState(_stateMachine, AccroState::GetInstance());
            return;
        }
    } else {
        // after 20s without pwr
        stabilization.ResetPID();
        if (_stateMachine->IsSafetyStateNeeded()) // Safety cut mngt: set safety cut
        {
            SetState(_stateMachine, SafetyState::GetInstance());
            return;
        }
    }

    SetState(_stateMachine, this);
}
