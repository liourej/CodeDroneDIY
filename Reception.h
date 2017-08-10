#define FLYING_MODE_ANGLE 0
#define FLYING_MODE_ACCRO 1

enum Mode { initialization, starting, safety, disarmed, accro, angle};

class Reception
{
    // Channel 1: Ailerons 1.09 to 1.90 ms
    // Channel 2: Prof 1.09 to 1.90 ms
    // Channel 3: Throttle 1.09 to 1.90 ms
    // Channel 4: Rudder 1.09 to 1.90 ms
  private:
    // PWM computation
    bool initialized = false;
    int channel = 0;
    float PWM_Start = 0;
    float PWM_Stop = 0;
    float PWM_Width = 0;
    int cPPM[7] = {0, 0, 0, 0, 0, 0, 0}; // 6 channels plus separation

  public:
    void PrintCmd(void) {
      Serial.print(F("Aile: ")); Serial.print(cPPM[0]); Serial.print(F("\tElev: ")); Serial.print(cPPM[1]);
      Serial.print(F("\tThrot: ")); Serial.print(cPPM[2]); Serial.print(F("\tRudd: ")); Serial.println(cPPM[3]);
      Serial.print(F("Switch1: ")); Serial.print(cPPM[4]); Serial.print(F("\tSwitch2: ")); Serial.println(cPPM[5]);
    }

    bool  IsReady() {
      return  initialized;
    };

    // Angle Mode:
    inline float GetAileronsAngle() {
      return -(map(cPPM[0], 1080, 1900, -MAX_ANGLE, MAX_ANGLE));
    };
    inline float GetElevatorAngle() {
      return (map(cPPM[1], 1080, 1900, -MAX_ANGLE, MAX_ANGLE));
    };

    // Accro mode:
    inline float GetAileronsSpeed() {
      return -(map(cPPM[0], 1080, 1900, -MAX_ROT_SPEED, MAX_ROT_SPEED));
    };
    inline float GetElevatorSpeed() {
      return (map(cPPM[1], 1080, 1900, -MAX_ROT_SPEED, MAX_ROT_SPEED));
    };
    inline int GetThrottle() {
      return map(cPPM[2], 1080, 1900, MIN_POWER, MAX_POWER);
    };
    inline int GetRudder() {
      return map(cPPM[3], 1080, 1900, -MAX_YAW_SPEED, MAX_YAW_SPEED);
    };
    inline int GetSwitchH() {
      if (cPPM[5] > 1500) return true;
      else return false;
    }; //1900 inter H en bas, 1090 inter H en haut


    inline int GetFlyingMode() {
      if (cPPM[4] > 1800)
        return disarmed;
      else if (cPPM[4] < 1200)
        return angle;
      else
        return accro;
    }; //G switch: pos0=1900, pos1=1500, pos2=1092

    inline void GetWidth(void) {
      PWM_Stop = micros();
      PWM_Width = PWM_Stop - PWM_Start;
      PWM_Start = PWM_Stop;

      if ( initialized ) {
        if ( channel < 7)
          cPPM[channel] = PWM_Width;
      }

      if ( PWM_Width > 4000 ) // If delay more than 4ms
      {
       channel = 0;
        initialized = true;
        //Serial.print("\t");Serial.print(cPPM[0]); Serial.print("\t");Serial.print(cPPM[1]); Serial.print("\t");Serial.print(cPPM[2]); Serial.print("\t");Serial.println(cPPM[3];Serial.println(cPPM[4]);
      } else
        channel++;
    }
};
