#ifndef ANGLESTATE_H_
#define ANGLESTATE_H_

#include "IState.h"
#include "StateMachine.h"

class AngleState : public IState {
public:
    static AngleState* GetInstance( )
    {
           static AngleState instance;
           return &instance;
    }

private:
    AngleState(){};
    AngleState(AngleState const&);// Don't Implement
    void operator=(AngleState const&); // Don't implement

public:
    void Run(StateMachine *_stateMachine, const float);
};
#endif
