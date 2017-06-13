#define usToTicks(_us)    (( clockCyclesPerMicrosecond()* _us) )     // converts microseconds to tick (assumes prescale of 8)  // 12 Aug 2009
#define ticksToUs(_ticks) (( (unsigned)_ticks )/ clockCyclesPerMicrosecond() ) // converts from ticks back to microseconds

class ESC
{
  public:
    int pin = 0;
    int PORTBPin = 0;
    float PWM = MIN_POWER;
    uint16_t ticks = 0;

  public:
    void attach (int _pin ) {
      pin = _pin;
      PORTBPin = _pin - 8;
      pinMode( _pin, OUTPUT);
    }; // set servo pin to output};
    
    inline void write (float _PWM) {
      if ( _PWM < MIN_POWER )
        _PWM = MIN_POWER;
      else if ( _PWM > MAX_POWER) // Check max power
      {
        Serial.println(F("WARNING, MAX POWER REACHED!!"));
        PWM = MAX_POWER;
      } else
        PWM = _PWM;

      // Last power check
      if ( PWM > MAX_POWER)
      {
        PWM = MAX_POWER;
      }
      ticks = usToTicks(PWM);
    };

    inline void Idle(){
      ticks = PWM = 0;
    }
    
    inline float GetPWM() {
      return PWM;
    };
    
    inline uint16_t GetTicks() {
      return ticks;
    };
};
