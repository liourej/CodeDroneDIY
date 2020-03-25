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
    // Power setup
    const int MIN_POWER = 1060;
    // Max power available to stabilize quardirotors - Set to 1860 to reach max
    // power
    const int MAX_POWER = 1860;
    const int MAX_THROTTLE_PERCENT = 100.0;
    // Threshold on max throttle command (percent of max motor power)
    uint16_t MAX_THROTTLE = MAX_POWER * (MAX_THROTTLE_PERCENT / 100.0);
    int IDLE_THRESHOLD = 1100;

  public:
    typeMotorSpeedControl motorsList[nbMotors];
    int currMotor = -1;
    void Init();
    void attach(int _id, int _pin); // set servo pin to output};
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
};

#endif // MOTORSPEEDCONTROL_H_