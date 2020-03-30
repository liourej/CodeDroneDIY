#include "AngleMode.h"
#include "AccroMode.h"
#include "Safety.h"
#ifndef UNIT_TEST
#include "../../stabilization/Stabilization.h"
#else
#include "../../../test/StabilizationStub.h"
#endif

#ifndef UNIT_TEST
extern Stabilization stabilization;
#else
extern StabilizationStub stabilization;
#endif

void AngleMode::Run(StateMachine *_stateMachine, const float _loopTimeSec) {
    if (stabilization.IsThrottleIdle()) {
        stabilization.ResetPID();
        if (_stateMachine->IsSafetyStateNeeded()) // Safety cut mngt: set safety cut
        {
            SetState(_stateMachine, Safety::GetInstance());
            return;
        }
    }
    _stateMachine->throttleWasHigh = true;
    stabilization.Angle(_loopTimeSec);

    // Allow to change flying mode during flight
    if (stabilization.GetFlyingMode() == accroMode) {
        CustomSerialPrint::println(F("Flying mode changed from angle to accro"));
        SetState(_stateMachine, AccroMode::GetInstance());
        return;
    }
}
