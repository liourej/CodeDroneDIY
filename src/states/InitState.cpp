#include "StateMachine.h"

extern StateMachine stateMachine;
extern float loopTimeSec;

// States
void *initState(const float) {
    stabilization.Idle();
    while (!stabilization.AreAttitudeOffsetsComputed())
        stabilization.AttitudeComputeOffsets();

    if (stabilization.GetFlyingMode() != disarmed)
        return (void *)&initState;
    else if (stabilization.AreAttitudeOffsetsComputed())
        return (void *)&startingState;

    return (void *)&initState;
}
