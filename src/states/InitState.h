#ifndef INITSTATE_H_
#define INITSTATE_H_

#include "IState.h"
#include "StateMachine.h"

class InitState : public IState {
public:
    static InitState* GetInstance( )
    {
           static InitState instance;
           return &instance;
    }

private:
    InitState(){};
    InitState(InitState const&);// Don't Implement
    void operator=(InitState const&); // Don't implement

public:
    void Run(StateMachine *_stateMachine, const float);
};
#endif
