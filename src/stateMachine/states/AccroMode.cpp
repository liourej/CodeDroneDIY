#include "AccroMode.h"
#include "AngleMode.h"
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

void AccroMode::Run(StateMachine *_stateMachine, const float _loopTimeSec) {
    if (stabilization.IsThrottleIdle()) {
        // after 5s without pwr
        stabilization.ResetPID();
        if (_stateMachine->IsSafetyStateNeeded()) // Safety cut mngt: set safety cut
        {
            SetState(_stateMachine, Safety::GetInstance());
            return;
        }
    }

    _stateMachine->throttleWasHigh = true;

    // Apply flight stabilization
    stabilization.Accro(_loopTimeSec);

    // Allow to change flying mode during flight
    if (stabilization.GetFlyingMode() == angleMode) {
        CustomSerialPrint::println(F("Flying mode changed from accro to angle"));
        SetState(_stateMachine, AngleMode::GetInstance());
        return;
    }
}
