#define usToTicks(_us)    (( clockCyclesPerMicrosecond()*_us) )     // converts microseconds to tick (assumes prescale of 8)  // 12 Aug 2009
#define ticksToUs(_ticks) (( (unsigned)_ticks )/ clockCyclesPerMicrosecond() ) // converts from ticks back to microseconds

#define ESCNb  4

typedef struct {
  int pin;
  float PWM;
  uint16_t ticks;
} typeESC;

enum ESCId{ESC0, ESC1, ESC2, ESC3};

class ESC
{
  public:
    // Power setup
    const uint8_t MIN_POWER = 1060;
    const uint8_t MAX_POWER = 1860; // Max power available to stabilize quardirotors - Set to 1860 to reach max power
    const uint8_t MAX_THROTTLE_PERCENT = 100.0;
    uint8_t MAX_THROTTLE = MAX_POWER*(MAX_THROTTLE_PERCENT/100.0); // Threshold on max throttle command (percent of max motor power)
    uint8_t IDLE_THRESHOLD = 1100;

  public:
    typeESC ESCList[4];
    int currESC = -1;
    void Init();

    void attach (int _id, int _pin ); // set servo pin to output};

    void write (int _id, float _PWM);

    void Idle();

    void SetPWM_f5(volatile uint16_t *TCNTn, volatile uint16_t* OCRnA);
};
