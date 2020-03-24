#include "InitState.h"
#include "StartingState.h"
#include "../Stabilization.h"

extern Stabilization stabilization;

void InitState::Run(StateMachine *_stateMachine, const float) {
    stabilization.Idle();
    while (!stabilization.AreAttitudeOffsetsComputed())
        stabilization.AttitudeComputeOffsets();

    if (stabilization.GetFlyingMode() != disarmed)
        SetState(_stateMachine,this);
    else if (stabilization.AreAttitudeOffsetsComputed())
        SetState(_stateMachine, StartingState::GetInstance());

    SetState(_stateMachine, this);
}
