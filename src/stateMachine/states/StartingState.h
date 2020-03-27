#ifndef STARTINGSTATE_H_
#define STARTINGSTATE_H_

#include "IState.h"
#include "../StateMachine.h"
#include "../../Singleton.h"

class StartingState : public Singleton<StartingState, IState> {
  public:
    int GetName() {
        return Mode::starting;
    }
    void Run(StateMachine *_stateMachine, const float);
};
#endif
