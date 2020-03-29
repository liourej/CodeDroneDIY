#ifndef MOTORSPEEDCONTROL_H_
#define MOTORSPEEDCONTROL_H_

#include "../../customLibs/CustomSerialPrint.h"

// converts microseconds to tick (assumes prescale of 8)
#define usToTicks(_us) ((clockCyclesPerMicrosecond() * _us))
// converts from ticks back to microseconds
#define ticksToUs(_ticks) (((unsigned)_ticks) / clockCyclesPerMicrosecond())

enum MotorId { Motor0, Motor1, Motor2, Motor3 };

class MotorsSpeedControl {
  private:
    static const int nbMotors = 4;
    const int MIN_POWER = 1060;
    const int MAX_POWER = 1860;             // Max pwr available. Set to 1860 to reach max
    const int MAX_THROTTLE_PERCENT = 100.0; // Percent to restrain max motor power
    uint16_t MAX_THROTTLE = MAX_POWER * (MAX_THROTTLE_PERCENT / 100.0); // Restrained max power
    int IDLE_THRESHOLD = 1100;
    static uint16_t motorsTicks[nbMotors];
    static int currMotor;

  public:
    void Init();
    void UpdateSpeed(int _id, float _PWM);
    void Idle();
    static void ApplySpeed(volatile uint16_t *TCNTn, volatile uint16_t *OCRnA);
    const int GetMotorsMaxPower() {
        return MAX_POWER;
    }
    const int GetMotorsMinPower() {
        return MIN_POWER;
    }
    const int GetMotorsMaxThrottlePercent() {
        return MAX_THROTTLE_PERCENT;
    }
    const int GetMotorsMaxThrottle() {
        return MAX_THROTTLE;
    }
    int GetMotorsIdleThreshold() {
        return IDLE_THRESHOLD;
    }

  private:
    void InitTimer1();
};

#endif // MOTORSPEEDCONTROL_H_