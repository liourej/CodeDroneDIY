#ifndef INITSTATE_H_
#define INITSTATE_H_

#include "IState.h"
#include "../StateMachine.h"
#include "../../Singleton.h"

class InitState : public Singleton<InitState, IState>{
  public:
    void Run(StateMachine *_stateMachine, const float);
};
#endif
