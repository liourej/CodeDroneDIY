#include "StartingState.h"
#include "AngleState.h"
#include "AccroState.h"

#ifndef UNIT_TEST
#include "../../stabilization/Stabilization.h"
#else
#include "../../stabilization/StabilizationStub.h"
#endif

#ifndef UNIT_TEST
extern Stabilization stabilization;
#else
extern StabilizationStub stabilization;
#endif

void StartingState::Run(StateMachine *_stateMachine, const float) {
    stabilization.Idle();
    int state = stabilization.GetFlyingMode();
    delay(500);
    if (state != stabilization.GetFlyingMode()) // Check it was not a transitory switch state
    {
        SetState(_stateMachine, StartingState::GetInstance());
        return;
    }
    if ((state == angle) || (state == accro)) {
        CustomSerialPrint::println(F("stateMachine.state != disarmed MODE"));
        // PrintSettings();
        if (state == angle)
            SetState(_stateMachine, AngleState::GetInstance());
        if (state == accro)
            SetState(_stateMachine, AccroState::GetInstance());
    } else {
        SetState(_stateMachine, this);
    }
}
