#include "RadioReception.h"

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
int RadioReception::GetAileronsAngle() {
    return -(map(cPPM[0], 1080, 1900, -MAX_ANGLE, MAX_ANGLE));
}
int RadioReception::GetElevatorAngle() {
    return (map(cPPM[1], 1080, 1900, -MAX_ANGLE, MAX_ANGLE));
}

// Accro mode:
int RadioReception::GetAileronsSpeed() {
    return -(map(cPPM[0], 1080, 1900, -MAX_ROT_SPEED, MAX_ROT_SPEED));
}
int RadioReception::GetElevatorSpeed() {
    return (map(cPPM[1], 1080, 1900, -MAX_ROT_SPEED, MAX_ROT_SPEED));
}
int RadioReception::GetThrottle(const int _minPower, const int _maxThrottle) {
    return map(cPPM[2], 1080, 1900, _minPower, _maxThrottle);
}

int RadioReception::GetRudder() {
    return map(cPPM[3], 1080, 1900, -MAX_YAW_SPEED, MAX_YAW_SPEED);
}
int RadioReception::GetSwitchH() {
    if (cPPM[5] > 1500) {
        return true;
    }
    return false;
} // 1900 inter H en bas, 1090 inter H en haut

int RadioReception::GetFlyingMode() {
    if (cPPM[4] > 1800)
        return disarmed;
    else if (cPPM[4] < 1200)
        return angle;
    else
        return accro;
} // G switch: pos0=1900, pos1=1500, pos2=1092

void RadioReception::GetWidth(void) {
    PWM_Stop = micros();
    PWM_Width = PWM_Stop - PWM_Start;
    PWM_Start = PWM_Stop;

    if (initialized) {
        if (channel < CHANNELS_NB)
            cPPM[channel] = PWM_Width;
    }

    if (PWM_Width > 4000) { // If delay more than 4ms, it is a new sequence
        channel = 0;
        initialized = true;
    } else if ((channel + 1) < CHANNELS_NB) {
        channel++;
    }
}
