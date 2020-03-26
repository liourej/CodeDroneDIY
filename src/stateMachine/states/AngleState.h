#ifndef ANGLESTATE_H_
#define ANGLESTATE_H_

#include "IState.h"
#include "../StateMachine.h"
#include "../../Singleton.h"

class AngleState : public Singleton<AngleState, IState>{
  public:
    void Run(StateMachine *_stateMachine, const float);
};
#endif
