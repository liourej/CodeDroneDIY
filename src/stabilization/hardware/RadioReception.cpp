#include "RadioReception.h"

// Initialize static members
unsigned long RadioReception::cPPM[CHANNELS_NB] = {
        0, 0, 0, 0, 0, 0,
};
bool RadioReception::initialized = false;
int RadioReception::nbSpacingEncountered = 0;
int RadioReception::channel = 0;
unsigned long RadioReception::PWM_Start = 0;
unsigned long RadioReception::PWM_Stop = 0;
unsigned long RadioReception::PWM_Width = 0;

bool RadioReception::Init() {
    attachInterrupt(0, &GetWidth, RISING); // Receiver interrupt on PD2 (INT0)

    CustomTime timeout;
    timeout.Init();
    while (!initialized) {
        CustomSerialPrint::println(F("RadioReception not ready, try again, please wait. "));
        delay(200);
        if (timeout.IsTimeout(2000)) {
            CustomSerialPrint::println(F("RadioReception - Timeout during initialization!!"));
            return false;
        }
    }
    return true;
}

void RadioReception::PrintCmd(void) {
    CustomSerialPrint::print(F("Aile: "));
    CustomSerialPrint::print(cPPM[0]);
    CustomSerialPrint::print(F(" Elev: "));
    CustomSerialPrint::print(cPPM[1]);
    CustomSerialPrint::print(F(" Throt: "));
    CustomSerialPrint::print(cPPM[2]);
    CustomSerialPrint::print(F(" Rudd: "));
    CustomSerialPrint::println(cPPM[3]);
    CustomSerialPrint::print(F("Switch1: "));
    CustomSerialPrint::print(cPPM[4]);
    CustomSerialPrint::print(F(" Switch2: "));
    CustomSerialPrint::println(cPPM[5]);
}

// Angle Mode:
long RadioReception::GetRollAngle() {
    return -(map(static_cast<long>(cPPM[0]), 1080, 1900, -MAX_ANGLE, MAX_ANGLE));
}
long RadioReception::GetPitchAngle() {
    return (map(cPPM[1], 1080, 1900, -MAX_ANGLE, MAX_ANGLE));
}

// Accro mode:
long RadioReception::GetRollSpeed() {
    return -(map(static_cast<long>(cPPM[0]), 1080, 1900, -MAX_ROT_SPEED, MAX_ROT_SPEED));
}
long RadioReception::GetPitchSpeed() {
    return (map(static_cast<long>(cPPM[1]), 1080, 1900, -MAX_ROT_SPEED, MAX_ROT_SPEED));
}
long RadioReception::GetThrottle(const int _minPower, const int _maxThrottle) {
    return map(static_cast<long>(cPPM[2]), 1080, 1900, _minPower, _maxThrottle);
}

long RadioReception::GetYawSpeed() {
    return map(static_cast<long>(cPPM[3]), 1080, 1900, -MAX_YAW_SPEED, MAX_YAW_SPEED);
}
bool RadioReception::GetSwitchH() {
    if (cPPM[5] > 1500) {
        return true;
    }
    return false;
} // 1900 inter H en bas, 1090 inter H en haut

int RadioReception::GetFlyingMode() {
    if (cPPM[4] > 1800)
        return disarmed;
    else if (cPPM[4] < 1200)
        return angleMode;
    else
        return accroMode;
} // G switch: pos0=1900, pos1=1500, pos2=1092

void RadioReception::GetWidth(void) {
    PWM_Stop = micros();
    PWM_Width = PWM_Stop - PWM_Start;
    PWM_Start = PWM_Stop;

    if (PWM_Width >= 4000) { // If delay more than 4ms, it is a new sequence
        if (nbSpacingEncountered >= 4)
            initialized = true; // Starting sequence encountered AND all channels received
        channel = 0;
        nbSpacingEncountered++;
        return;
    }

    if (nbSpacingEncountered == 0) // When application starts, it must wait for a starting sequence
        return;

    if (channel < CHANNELS_NB) {
        cPPM[channel] = PWM_Width;
        channel++;
    }
}
