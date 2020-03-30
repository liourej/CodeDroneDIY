#include "Initializing.h"
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

void Initializing::Run(StateMachine *_stateMachine, const float) {
    stabilization.Idle();
    while (!stabilization.AreAttitudeOffsetsComputed())
        stabilization.AttitudeComputeOffsets();

    if ((stabilization.GetFlyingMode() == disarmed) && stabilization.AreAttitudeOffsetsComputed())
        SetState(_stateMachine, Disarmed::GetInstance());
}
