#include "SafetyState.h"
#include "DisarmedState.h"
#include "../Stabilization.h"

extern Stabilization stabilization;

void SafetyState::Run(StateMachine *_stateMachine, const float){
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
