#include "DisarmedState.h"
#include "AccroState.h"
#include "AngleState.h"
#include "../Stabilization.h"

extern Stabilization stabilization;

void DisarmedState::Run(StateMachine *_stateMachine, const float _loopTimeSec) {
    stabilization.Idle();
    _stateMachine->ActivateBuzzer(120);
    int state = stabilization.GetFlyingMode();
    delay(500);
    // Check it was not a transitory switch state
    if (state != stabilization.GetFlyingMode())
        SetState(_stateMachine, DisarmedState::GetInstance());
    if (state != disarmed) {
        _stateMachine->throttleWasHigh = true;
        if (state == angle) {
            Serial.println(F("ANGLE MODE"));
            SetState(_stateMachine, AngleState::GetInstance());
        } else if (state == accro) {
            Serial.println(F("ACCRO MODE"));
            SetState(_stateMachine, AccroState::GetInstance());
        }
    }
      SetState(_stateMachine, this);
}
