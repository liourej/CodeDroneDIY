#ifndef STATE_H_
#define STATE_H_

class StateMachine;

class IState {

public:
  void Run(StateMachine* _stateMachine, const float){};

protected:
  void SetState(StateMachine *_stateMachine, IState *_newState);
};
#endif
