#ifndef STATEMACHINE_H_
#define STATEMACHINE_H_

#include "Time.h"
#include "Reception.h"
#include "Stabilization.h"

extern Reception Rx;
extern Stabilization stabilization;

// States functions
void *initState();
void *startingState();
void *angleState();
void *accroState();
void *safetyState();
void *disarmedState();

class StateMachine {
  private:
    // Buzzer for lost model alarm
    const int BUZZER_PIN = 7;
    const int delayThresholdSec = 5; // (s)
    Time elapsedTime;
    Time timeBuzzer;
    bool setBuzzer = false;

  public:
    bool throttleWasHigh = true;
    void Init();

    // Activate buzzer after x minutes of power idle
    void ActivateBuzzer(int _duration);
    void Print();

    // Auto Disarm when throttle is idle since a long period
    bool IsSafetyStateNeeded();
};

#endif // STATEMACHINE_H_
