#ifndef ANGLESTATE_H_
#define ANGLESTATE_H_

#include "IState.h"
#include "../StateMachine.h"
#include "../../Singleton.h"

class AngleMode : public Singleton<AngleMode, IState> {
  public:
    int GetName() {
        return Mode::angleMode;
    }
    void Run(StateMachine *_stateMachine, const float);
};
#endif
