#ifndef DISARMEDSTATE_H_
#define DISARMEDSTATE_H_

#include "IState.h"
#include "../StateMachine.h"
#include "../../Singleton.h"

class DisarmedState : public Singleton<DisarmedState, IState> {
  public:
    int GetName() {
        return Mode::disarmed;
    }
    void Run(StateMachine *_stateMachine, const float);
};
#endif
