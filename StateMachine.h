extern int g_Flyingstate;
extern Reception Rx;

class StateMachine {
  private:
    const int delayThreshold = 5;

    Time elapsedTime;
  public:
    int state = initialization;
    int statePrev = initialization;
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
      elapsedTime.Init();
    }

    void RefreshState() {
      if ( throttleWasHigh ) {
        Serial.println(F("Throttle just setted low!"));
        Init();
        throttleWasHigh = false;
      } else if ( (state != safety) &&
                  (state != disarmed) &&
                  (elapsedTime.GetExecutionTime() > delayThreshold) )
      {
        state = safety;
        Serial.print(delayThreshold); Serial.println(F(" sec without power, system DISARMED!"));
      }
    };
};
