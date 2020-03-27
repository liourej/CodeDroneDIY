#ifndef RADIORECEPTION_H_
#define RADIORECEPTION_H_

#include "Arduino.h"
#include "../../customLibs/CustomSerialPrint.h"

enum Mode { initialization, starting, safety, disarmed, accro, angle };

class RadioReception {
  public:
    // RadioReception setup
    const float MAX_ANGLE = 45;      // (°) Max roll and pitch angles reachable in angle mode
    const float MAX_ROT_SPEED = 135; // (°/s) Max roll and pitch speed in accro mode
    const float MAX_YAW_SPEED = 135; // (°/s) Max yaw speed in accro and angle modes

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
    static const uint8_t CHANNELS_NB = 7;
    int cPPM[CHANNELS_NB] = {0, 0, 0, 0, 0, 0, 0}; // 6 channels plus separation

  public:
    void PrintCmd(void);

    bool IsReady() {
        return initialized;
    }

    // Angle Mode:
    int GetAileronsAngle();
    int GetElevatorAngle();

    // Accro mode:
    int GetAileronsSpeed();
    int GetElevatorSpeed();
    int GetThrottle(const int _minPower, const int _maxThrottle);

    int GetRudder();
    int GetSwitchH(); // 1900 inter H en bas, 1090 inter H en haut

    int GetFlyingMode(); // G switch: pos0=1900, pos1=1500, pos2=1092

    void GetWidth(void);
};

#endif // RADIORECEPTION_H_
