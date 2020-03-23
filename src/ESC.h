#ifndef ESC_H_
#define ESC_H_

// converts microseconds to tick (assumes prescale of 8)  // 12 Aug 2009
#define usToTicks(_us) ((clockCyclesPerMicrosecond() * _us))
// converts from ticks back to microseconds
#define ticksToUs(_ticks) (((unsigned)_ticks) / clockCyclesPerMicrosecond())

//#define ESCNb 4

typedef struct {
    int pin;
    float PWM;
    uint16_t ticks;
} typeESC;

enum ESCId { ESC0, ESC1, ESC2, ESC3 };

class ESC {
  private:
    static const int nbESC = 4;
    // Power setup
    const int MIN_POWER = 1060;
    // Max power available to stabilize quardirotors - Set to 1860 to reach max
    // power
    const int MAX_POWER = 1860;
    const int MAX_THROTTLE_PERCENT = 100.0;
    // Threshold on max throttle command (percent of max motor power)
    uint8_t MAX_THROTTLE = MAX_POWER * (MAX_THROTTLE_PERCENT / 100.0);
    int IDLE_THRESHOLD = 1100;

  public:
    typeESC ESCList[nbESC];
    int currESC = -1;
    void Init();
    void attach(int _id, int _pin); // set servo pin to output};
    void write(int _id, float _PWM);
    void Idle();
    void SetPWM_f5(volatile uint16_t *TCNTn, volatile uint16_t *OCRnA);
    const int GetESCsMaxPower() {
        return MAX_POWER;
    }
    const int GetESCsMinPower() {
        return MIN_POWER;
    }
    const int GetESCsMaxThrottlePercent() {
        return MAX_THROTTLE_PERCENT;
    }
    const int GetESCsMaxThrottle() {
        return MAX_THROTTLE;
    }
    int GetESCIdleThreshold() {
        return IDLE_THRESHOLD;
    }
};

#endif // ESC_H_
