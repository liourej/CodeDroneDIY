#define usToTicks(_us)    (( clockCyclesPerMicrosecond()*_us) )     // converts microseconds to tick (assumes prescale of 8)  // 12 Aug 2009
#define ticksToUs(_ticks) (( (unsigned)_ticks )/ clockCyclesPerMicrosecond() ) // converts from ticks back to microseconds

#define ESCNb  4

typedef struct {
  int pin;
  float PWM;
  uint16_t ticks;
} typeESC;

class ESC
{
  public:
    typeESC ESCList[4];
    int currESC = -1;
    void Init() {
      attach(0, 4);
      attach(1, 5);
      attach(2, 6);
      attach(3, 7);
      Idle();
    }

    void attach (int _id, int _pin ) {
      ESCList[_id].pin = _pin;
      pinMode( _pin, OUTPUT);
    }; // set servo pin to output};

    void write (int _id, float _PWM) {
      if ( _PWM < MIN_POWER )
        ESCList[_id].PWM = MIN_POWER;
      else if ( _PWM > MAX_POWER) // Check max power
      {
        Serial.println(F("WARNING, MAX POWER REACHED!!"));
        ESCList[_id].PWM = MAX_POWER;
      } else
        ESCList[_id].PWM = _PWM;

      ESCList[_id].ticks = usToTicks(_PWM);
    };

    void Idle() {
      for (int id = 0; id < 4; id++) {
        ESCList[id].ticks = usToTicks( MIN_POWER);
        ESCList[id].PWM = MIN_POWER;
      }
    }
    void SetPWM_f5(volatile uint16_t *TCNTn, volatile uint16_t* OCRnA) {
      static bool firstLoop = true;
      if ( firstLoop) {
        currESC = 0;
        PORTD = B00010000;
        firstLoop = false;
      }

      if ( currESC == 0 ) {
        PORTD ^= B00110000; // Reset pin PD4 and set pin PD5 using XOR
        *OCRnA = ESCList[currESC + 1].ticks;
      } else if ( currESC == 1 ) {
        PORTD ^= B01100000; // Reset pin PD5 and set pin PD6 using XOR
        *OCRnA = ESCList[currESC + 1].ticks;
      } else if ( currESC == 2 ) {
        PORTD ^= B11000000; // Reset pin PD6 and set pin 7 using XOR
        *OCRnA = ESCList[currESC + 1].ticks;
      } else {
        PORTD ^= B10010000; // Reset pin PD7 and set pin PD4 using XOR

        currESC = -1;
        *OCRnA = ESCList[currESC + 1].ticks;
      }
      *TCNTn = 0; // Reset timer
      currESC++;
    }
};
