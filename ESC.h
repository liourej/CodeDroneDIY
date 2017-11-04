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
    typeESC ESCList[4];
    int currESC = -1;
    void Init();

    void attach (int _id, int _pin ); // set servo pin to output};

    void write (int _id, float _PWM);

    void Idle();

    void SetPWM_f5(volatile uint16_t *TCNTn, volatile uint16_t* OCRnA);
};
