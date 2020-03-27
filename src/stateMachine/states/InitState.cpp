#include "InitState.h"
#include "StartingState.h"
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

void InitState::Run(StateMachine *_stateMachine, const float) {
    stabilization.Idle();
    while (!stabilization.AreAttitudeOffsetsComputed())
        stabilization.AttitudeComputeOffsets();

    if (stabilization.GetFlyingMode() != disarmed)
        SetState(_stateMachine, this);
    else if (stabilization.AreAttitudeOffsetsComputed())
        SetState(_stateMachine, StartingState::GetInstance());
    else
        SetState(_stateMachine, this);
}
