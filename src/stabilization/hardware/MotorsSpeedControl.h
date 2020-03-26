#ifndef MOTORSPEEDCONTROL_H_
#define MOTORSPEEDCONTROL_H_

// converts microseconds to tick (assumes prescale of 8)  // 12 Aug 2009
#define usToTicks(_us) ((clockCyclesPerMicrosecond() * _us))
// converts from ticks back to microseconds
#define ticksToUs(_ticks) (((unsigned)_ticks) / clockCyclesPerMicrosecond())

typedef struct {
    int pin;
    float PWM;
    uint16_t ticks;
} typeMotorSpeedControl;

enum MotorId { Motor0, Motor1, Motor2, Motor3 };

class MotorsSpeedControl {
  private:
    static const int nbMotors = 4;
    const int MIN_POWER = 1060;
    const int MAX_POWER = 1860; // Max pwr available. Set to 1860 to reach max
    const int MAX_THROTTLE_PERCENT = 100.0; // Percent to restrain max motor power
    uint16_t MAX_THROTTLE = MAX_POWER * (MAX_THROTTLE_PERCENT / 100.0); // Restrained max power
    int IDLE_THRESHOLD = 1100;
    typeMotorSpeedControl motorsList[nbMotors];
    int currMotor = -1;

  public:
    void Init();
    void write(int _id, float _PWM);
    void Idle();
    void SetMotorsSpeed(volatile uint16_t *TCNTn, volatile uint16_t *OCRnA);
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
    void attach(int _id, int _pin); // set servo pin to output};
};

#endif // MOTORSPEEDCONTROL_H_