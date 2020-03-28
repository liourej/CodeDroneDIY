#include "DisarmedState.h"
#include "AccroState.h"
#include "AngleState.h"
#ifndef UNIT_TEST
#include "../../stabilization/Stabilization.h"
#else
#include "../../../test/StabilizationStub.h"
#endif

#ifndef UNIT_TEST
extern Stabilization stabilization;
#else
extern StabilizationStub stabilization;
#endif

void DisarmedState::Run(StateMachine *_stateMachine, const float _loopTimeSec) {
    stabilization.Idle();
    int state = stabilization.GetFlyingMode();
    delay(500);
    // Check it was not a transitory switch state
    if (state != stabilization.GetFlyingMode()) {
        SetState(_stateMachine, DisarmedState::GetInstance());
        return;
    }
    if (state != disarmed) {
        _stateMachine->throttleWasHigh = true;
        if (state == angle) {
            CustomSerialPrint::println(F("ANGLE MODE"));
            SetState(_stateMachine, AngleState::GetInstance());
            return;
        } else if (state == accro) {
            CustomSerialPrint::println(F("ACCRO MODE"));
            SetState(_stateMachine, AccroState::GetInstance());
            return;
        }
    }
    SetState(_stateMachine, this);
}
