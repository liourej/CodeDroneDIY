#ifndef Disarmed_H_
#define Disarmed_H_

#include "IState.h"
#include "../StateMachine.h"
#include "../../Singleton.h"

class Disarmed : public Singleton<Disarmed, IState> {
  public:
    int GetName() {
        return Mode::disarmed;
    }
    void Run(StateMachine *_stateMachine, const float);
};
#endif
