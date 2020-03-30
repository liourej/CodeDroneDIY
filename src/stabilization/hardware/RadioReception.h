#ifndef RADIORECEPTION_H_
#define RADIORECEPTION_H_

#include "../../customLibs/CustomSerialPrint.h"
#include "../../customLibs/CustomTime.h"

enum Mode { initialization, safety, disarmed, accroMode, angleMode };

class RadioReception {
  public:
    // RadioReception setup
    static constexpr float MAX_ANGLE = 45; // (°) Max roll and pitch angles reachable in angle mode
    static constexpr float MAX_ROT_SPEED = 135; // (°/s) Max roll and pitch speed in accro mode
    static constexpr float MAX_YAW_SPEED = 135; // (°/s) Max yaw speed in accro and angle modes

    // Channel 1: Ailerons 1.09 to 1.90 ms
    // Channel 2: Prof 1.09 to 1.90 ms
    // Channel 3: Throttle 1.09 to 1.90 ms
    // Channel 4: Rudder 1.09 to 1.90 ms

  private:
    // PWM computation
    static bool initialized;
    static int nbSpacingEncountered;
    static int channel;
    static unsigned long PWM_Start;
    static unsigned long PWM_Stop;
    static unsigned long PWM_Width;
    static constexpr uint8_t CHANNELS_NB = 6; // 6 channels without separation pulse
    static unsigned long cPPM[CHANNELS_NB];   // 6 channels without separation pulse

  public:
    static bool Init();
    static void PrintCmd(void);

    // Angle Mode:
    static long GetRollAngle();
    static long GetPitchAngle();

    // Accro mode:
    static long GetRollSpeed();
    static long GetPitchSpeed();
    static long GetThrottle(const int _minPower, const int _maxThrottle);

    static long GetYawSpeed();
    static bool GetSwitchH(); // 1900 inter H en bas, 1090 inter H en haut

    static int GetFlyingMode(); // G switch: pos0=1900, pos1=1500, pos2=1092

    static void GetWidth(void);
};

#endif // RADIORECEPTION_H_
