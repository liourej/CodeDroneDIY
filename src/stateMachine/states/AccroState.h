#ifndef ACCROSTATE_H_
#define ACCROSTATE_H_

#include "IState.h"
#include "../StateMachine.h"
#include "../../Singleton.h"

class AccroState : public Singleton<AccroState, IState>{
  public:
    void Run(StateMachine *_stateMachine, const float);
};
#endif
