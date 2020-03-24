#ifndef SAFETYSTATE_H_
#define SAFETYSTATE_H_

#include "IState.h"
#include "../StateMachine.h"

class SafetyState : public IState {
  public:
    static SafetyState *GetInstance() {
        static SafetyState instance;
        return &instance;
    }

  private:
    SafetyState(){};
    SafetyState(SafetyState const &);    // Don't Implement
    void operator=(SafetyState const &); // Don't implement
  public:
    void Run(StateMachine *_stateMachine, const float);
};
#endif
