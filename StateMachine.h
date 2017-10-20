extern int g_Flyingstate;
extern Reception Rx;

class StateMachine {
  private:
    const int delayThreshold = 5; // (s)

    Time elapsedTime;
    Time timeBuzzer;

    bool setBuzzer = false;
  public:
    int state = initialization;
    bool throttleWasHigh = true;
    int printedState = -1;

    void Print() {
      if( state == printedState )
      return;
      switch (state) {
        case initialization:
          Serial.println(F("initialization"));
          break;
        case starting:
          Serial.println(F("starting"));
          break;
        case safety:
          Serial.println(F("safety"));
          break;
        case disarmed:
          Serial.println(F("disarmed"));
          break;
        case accro:
          Serial.println(F("accro"));
          break;
        case angle:
          Serial.println(F("angle"));
          break;
      }
      printedState = state;

    }
    void Init() {
      elapsedTime.Init(0);
      timeBuzzer.Init(0);
      setBuzzer = false;
    }

    void ActivateBuzzer(int _duration) {

      if( setBuzzer ){
        Time time;
        time.Init(0);
        while ( (time.GetExecutionTime(0) * 1000) < _duration) {
          digitalWrite(12, HIGH);
          digitalWrite(13, HIGH);
          delayMicroseconds(1800);
          digitalWrite(12, LOW);
          digitalWrite(13, LOW);
          delay(10);
          wdt_reset();
          Serial.println(F("BUZZZZZ"));
        }
      }else if( timeBuzzer.GetExecutionTime(0) > 120 ) // Activate buzzer after 2 minutes
        setBuzzer = true;
    }

    void RefreshState() {
      if ( throttleWasHigh ) {
        Serial.println(F("Throttle just setted low!"));
        Init();
        throttleWasHigh = false;
      } else if ( (state != safety) &&
                  (state != disarmed) &&
                  (elapsedTime.GetExecutionTime(0) > delayThreshold) ) // (s)
      {
        state = safety;
        Serial.print(delayThreshold); Serial.println(F(" sec without power, system DISARMED!"));
      }
    };
};
