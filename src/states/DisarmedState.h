#ifndef DISARMEDSTATE_H_
#define DISARMEDSTATE_H_

#include "IState.h"
#include "StateMachine.h"

class DisarmedState : public IState {
public:
    static DisarmedState* GetInstance( )
    {
           static DisarmedState instance;
           return &instance;
    }

private:
    DisarmedState(){};
    DisarmedState(DisarmedState const&);// Don't Implement
    void operator=(DisarmedState const&); // Don't implement

public:
    void Run(StateMachine *_stateMachine, const float);
};
#endif
