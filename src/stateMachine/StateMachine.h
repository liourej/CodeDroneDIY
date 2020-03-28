#ifndef STATEMACHINE_H_
#define STATEMACHINE_H_

#include "../customLibs/CustomTime.h"
#include "../stabilization/hardware/RadioReception.h"
#ifndef UNIT_TEST
#include "../stabilization/Stabilization.h"
#else
#include "../../test/StabilizationStub.h"
#endif
#include "states/IState.h"
#include "states/InitState.h"

class StateMachine {
  private:
    const int delayThresholdSec = 5; // (s)
    CustomTime elapsedTime;
    IState *currentState;

  public:
    bool throttleWasHigh = true;
    void Init();

     // Auto Disarm when throttle is idle since a long period
     bool IsSafetyStateNeeded();

     void SetState(IState *_newState) {
         currentState = _newState;
     }

     int GetStateName()
     {
       return currentState->GetName();
     }

     void Run(const float _loopTimeSec) {
         currentState->Run(this, _loopTimeSec);
     }

   private:
     void Print();
};

#endif // STATEMACHINE_H_
