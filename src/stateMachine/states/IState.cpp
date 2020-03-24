#include "IState.h"
#include "../StateMachine.h"

void IState::SetState(StateMachine *_stateMachine, IState *_newState) {
    _stateMachine->SetState(_newState);
}
