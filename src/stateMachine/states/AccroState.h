#ifndef ACCROSTATE_H_
#define ACCROSTATE_H_

/* #include <string.h> */

#include "IState.h"
#include "../StateMachine.h"
#include "../../Singleton.h"

class AccroState : public Singleton<AccroState, IState> {
  public:
    virtual int GetName() {
        return Mode::accro;
    }
    virtual void Run(StateMachine *_stateMachine, const float);
};
#endif
