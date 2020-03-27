#include "SafetyState.h"
#include "DisarmedState.h"
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

void SafetyState::Run(StateMachine *_stateMachine, const float) {
    stabilization.Idle();
    _stateMachine->ActivateBuzzer(120);
    stabilization.GetFlyingMode();
    if (stabilization.GetFlyingMode() != disarmed) {
        stabilization.Idle();
        SetState(_stateMachine, this);
    } else {
        SetState(_stateMachine, DisarmedState::GetInstance());
    }
}
