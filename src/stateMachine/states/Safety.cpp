#include "Safety.h"
#include "Disarmed.h"
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

void Safety::Run(StateMachine *_stateMachine, const float) {
    stabilization.Idle();
    stabilization.GetFlyingMode();
    if (stabilization.GetFlyingMode() != disarmed) {
        stabilization.Idle();
    } else {
        SetState(_stateMachine, Disarmed::GetInstance());
    }
}

