#ifndef ISTATE_H_
#define ISTATE_H_

#include <string.h>

class StateMachine;

class IState {
  public:
    virtual int GetName();
    virtual void Run(StateMachine *_stateMachine, const float);

  protected:
    void SetState(StateMachine *_stateMachine, IState *_newState);
};
#endif
