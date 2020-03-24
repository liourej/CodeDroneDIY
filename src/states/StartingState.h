#ifndef STARTINGSTATE_H_
#define STARTINGSTATE_H_

#include "IState.h"
#include "StateMachine.h"

class StartingState : public IState {
public:
    static StartingState* GetInstance( )
    {
           static StartingState instance;
           return &instance;
    }

private:
    StartingState(){};
    StartingState(StartingState const&);// Don't Implement
    void operator=(StartingState const&); // Don't implement

public:
    void Run(StateMachine *_stateMachine, const float);
};
#endif
