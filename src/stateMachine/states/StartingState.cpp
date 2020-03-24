#include "StartingState.h"
#include "AngleState.h"
#include "AccroState.h"

extern Stabilization stabilization;

void StartingState::Run(StateMachine *_stateMachine, const float) {
    stabilization.Idle();
    int state = stabilization.GetFlyingMode();
    delay(500);
    if (state != stabilization.GetFlyingMode()) // Check it was not a transitory switch state
        SetState(_stateMachine, StartingState::GetInstance());
    if ((state == angle) || (state == accro)) {
        Serial.println(F("stateMachine.state != disarmed MODE"));
        // PrintSettings();
        if (state == angle)
            SetState(_stateMachine, AngleState::GetInstance());
        if (state == accro)
            SetState(_stateMachine, AccroState::GetInstance());
    }
    SetState(_stateMachine, this);
}
