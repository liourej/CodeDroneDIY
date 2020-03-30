#include "AngleMode.h"
#include "AccroMode.h"
#include "Disarmed.h"

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

void Disarmed::Run(StateMachine *_stateMachine, const float) {
    stabilization.Idle();
    int state = stabilization.GetFlyingMode();
    delay(500);
    if (state != stabilization.GetFlyingMode()) // Check it was not a transitory switch state
        return;

    if (state == angleMode) {
        SetState(_stateMachine, AngleMode::GetInstance());
        return;
    } else if (state == accroMode) {
        SetState(_stateMachine, AccroMode::GetInstance());
        return;
    }
}
