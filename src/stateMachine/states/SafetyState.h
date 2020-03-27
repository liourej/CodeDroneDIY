#ifndef SAFETYSTATE_H_
#define SAFETYSTATE_H_

#include "IState.h"
#include "../StateMachine.h"
#include "../../Singleton.h"

class SafetyState : public Singleton<SafetyState, IState> {
  public:
    int GetName() {
        return Mode::safety;
    }
    void Run(StateMachine *_stateMachine, const float);
};
#endif
