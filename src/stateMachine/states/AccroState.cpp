#include "AccroState.h"
#include "AngleState.h"
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

void AccroState::Run(StateMachine *_stateMachine, const float _loopTimeSec) {
    if (!stabilization.IsThrottleIdle()) {
        _stateMachine->throttleWasHigh = true;

        stabilization.Accro(_loopTimeSec);

        // Allow to change flying mode during flight
        if (stabilization.GetFlyingMode() == angle) {
            CustomSerialPrint::println(F("Flying mode changed from accro to angle"));
            SetState(_stateMachine, AngleState::GetInstance());
            return;
        }
    } else {
        // after 5s without pwr
        stabilization.ResetPID();
        if (_stateMachine->IsSafetyStateNeeded()) // Safety cut mngt: set safety cut
        {
            SetState(_stateMachine, SafetyState::GetInstance());
            return;
        }
    }
    SetState(_stateMachine, this);
}
