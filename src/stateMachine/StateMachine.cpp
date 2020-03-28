#include "StateMachine.h"

void StateMachine::Init() {
    elapsedTime.Init();
    SetState(InitState::GetInstance());
}

// Auto disarm when throttle is idle since a long period
bool StateMachine::IsSafetyStateNeeded() {
    if (throttleWasHigh) {
        CustomSerialPrint::println(F("Throttle just setted low!"));
        Init();
        throttleWasHigh = false;
    } else if (elapsedTime.GetExecutionTimeSeconds() > delayThresholdSec) {
        CustomSerialPrint::print(delayThresholdSec);
        CustomSerialPrint::println(F(" sec without power, system DISARMED!"));
        return true;
    }
    return false;
}
