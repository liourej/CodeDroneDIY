#ifndef SAFETYSTATE_H_
#define SAFETYSTATE_H_

#include "IState.h"
#include "../StateMachine.h"
#include "../../Singleton.h"

class Safety : public Singleton<Safety, IState> {
  public:
    int GetName() {
        return Mode::safety;
    }
    void Run(StateMachine *_stateMachine, const float);
};
#endif
