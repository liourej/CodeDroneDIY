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
