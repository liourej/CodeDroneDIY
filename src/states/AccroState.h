#ifndef ACCROSTATE_H_
#define ACCROSTATE_H_

#include "IState.h"
#include "StateMachine.h"

class AccroState : public IState {
public:
    static AccroState* GetInstance( )
    {
           static AccroState instance;
           return &instance;
    }

private:
    AccroState(){};
    AccroState(AccroState const&);// Don't Implement
    void operator=(AccroState const&); // Don't implement

public:
    void Run(StateMachine *_stateMachine, const float);
};
#endif
