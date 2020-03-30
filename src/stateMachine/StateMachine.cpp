#include "StateMachine.h"

void StateMachine::Init() {
    elapsedTime.Init();
    SetState(Initializing::GetInstance());
}

// Auto disarm when throttle is idle since a long period
bool StateMachine::IsSafetyStateNeeded() {
    if (throttleWasHigh) {
        CustomSerialPrint::println(F("Throttle just setted low!"));
        elapsedTime.Init();
        throttleWasHigh = false;
        return false;
    } else if (elapsedTime.IsTimeout(delayThresholdSec * 1000)) {
        return true;
    }
    return true;
}
