#ifdef UNIT_TEST
#ifndef STABILIZATIONSTUB_H_
#define STABILIZATIONSTUB_H_

#include "../src/customLibs/CustomMath.h"
#include "../src/stabilization/hardware/RadioReception.h"

class StabilizationStub : public CustomMath {
  private:
    int callNb = 0;
 
  public:
    void SetMotorsPwrXConfig(){};
    void Init() {
    }
    void Idle(){};
    void Accro(float _loopTimeSec){};
    void Angle(float _loopTimeSec){};
    void ResetPID(){};
    void SetMotorsSpeed(volatile uint16_t *TCNTn, volatile uint16_t *OCRnA);
    int GetMotorsMaxPower() {
        return 1080;
    }
    int GetMotorsMinPower() {
        return 1080;
    }
    int GetMotorsMaxThrottle() {
        return 1080;
    }
    int GetFlyingMode() {
        switch (callNb) {
        case 0:
            callNb++;
            return Mode::disarmed;
            break;
        case 1:
        case 2:
            callNb++;
            return Mode::angleMode;
            break;
        case 3:
            callNb++;
            return Mode::accroMode;
            break;
        case 4:
            callNb++;
            return Mode::disarmed;
            break;
        }
        return Mode::disarmed;
    }
    int GetMotorsIdleThreshold() {
        return 1080;
    }
    void AttitudeComputeOffsets() {
    }
    void ComputeRxImpulsionWidth() {
    }

    int GetThrottle() {
        return 1100;
    }

    bool IsThrottleIdle() {
        return false;
    }

    bool AreAttitudeOffsetsComputed() {
        return true;
    }
};
#endif // STABILIZATIONSTUB_H_
#endif
