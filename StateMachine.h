extern int g_FlyingMode;
extern Reception Rx;

class StateMachine {
  private:
    const int delayThreshold = 5;

    Time elapsedTime;
  public:
    int  mode = disarmed;
    int modePrev = disarmed;
    bool throttleWasHigh = true;

    void Init() {
      elapsedTime.Init();
    }

    void RefreshState(){
      mode = Rx.GetFlyingMode();
      if ( throttleWasHigh ) {
        Serial.println(F("Throttle just setted low!"));
        Init();
        throttleWasHigh = false;
      } else if ( (mode != safety) &&
                  (mode != disarmed) &&
                  (elapsedTime.GetExecutionTime() > delayThreshold) )
      {
        mode = safety;
        Serial.print(delayThreshold); Serial.println(F(" sec without power, system DISARMED!"));
      }
      else if ( (mode == safety) || (mode == disarmed )) {
        ArmingSequence();
        throttleWasHigh = true;
      }
    };

    void ArmingSequence() {
      mode = Rx.GetFlyingMode();
      if ( mode != disarmed )
        Serial.println(F("Disarm before continue"));

      while (mode != disarmed ) {
        ESC0.Idle();
        ESC1.Idle();
        ESC2.Idle();
        ESC3.Idle();
        mode = Rx.GetFlyingMode();
        delay(200);
        wdt_reset();
      }

      Serial.println(F("Select flying mode"));

      while (mode == disarmed) {
        // Wait 2 sec to let user select the mode using the 3 positions switch
        for (int i = 0; i < 10; i++) {
          delay(200);
          wdt_reset();
        }
        mode = Rx.GetFlyingMode();
        if ( !throttleWasHigh && (mode != modePrev) )
        {
          mode  = disarmed;
          Serial.println(F("Choose same mode than previous used"));
        }
        ESC0.Idle();
        ESC1.Idle();
        ESC2.Idle();
        ESC3.Idle();
        wdt_reset();
      }
      modePrev = mode;
    };
};
