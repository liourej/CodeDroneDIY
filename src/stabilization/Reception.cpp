#include "Reception.h"

void Reception::PrintCmd(void) {
    Serial.print(F("Aile: "));
    Serial.print(cPPM[0]);
    Serial.print(F(" Elev: "));
    Serial.print(cPPM[1]);
    Serial.print(F(" Throt: "));
    Serial.print(cPPM[2]);
    Serial.print(F(" Rudd: "));
    Serial.println(cPPM[3]);
    Serial.print(F("Switch1: "));
    Serial.print(cPPM[4]);
    Serial.print(F(" Switch2: "));
    Serial.println(cPPM[5]);
}

// Angle Mode:
int Reception::GetAileronsAngle() {
    return -(map(cPPM[0], 1080, 1900, -MAX_ANGLE, MAX_ANGLE));
}
int Reception::GetElevatorAngle() {
    return (map(cPPM[1], 1080, 1900, -MAX_ANGLE, MAX_ANGLE));
}

// Accro mode:
int Reception::GetAileronsSpeed() {
    return -(map(cPPM[0], 1080, 1900, -MAX_ROT_SPEED, MAX_ROT_SPEED));
}
int Reception::GetElevatorSpeed() {
    return (map(cPPM[1], 1080, 1900, -MAX_ROT_SPEED, MAX_ROT_SPEED));
}
int Reception::GetThrottle(const int _minPower, const int _maxThrottle) {
    return map(cPPM[2], 1080, 1900, _minPower, _maxThrottle);
}

int Reception::GetRudder() {
    return map(cPPM[3], 1080, 1900, -MAX_YAW_SPEED, MAX_YAW_SPEED);
}
int Reception::GetSwitchH() {
    if (cPPM[5] > 1500) {
        return true;
    } else {
        return false;
    }
} // 1900 inter H en bas, 1090 inter H en haut

int Reception::GetFlyingMode() {
    if (cPPM[4] > 1800)
        return disarmed;
    else if (cPPM[4] < 1200)
        return angle;
    else
        return accro;
} // G switch: pos0=1900, pos1=1500, pos2=1092

void Reception::GetWidth(void) {
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
