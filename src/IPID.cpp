#include "IPID.h"

void IPID::Reset() {
    errorPrev = 0;
    integrator = 0;
}

int IPID::ComputeCorrection(float _cmd, float _pos, float _loopTime) {
    error = _cmd - _pos;
    integrator = integrator + error;
    int correction =
            (int)(G * (Kp * error + Kd * ((error - errorPrev) / (_loopTime)) + Ki * integrator));

    errorPrev = error;

    // Correction in us
    return correction;
}

void IPID::PrintGains(void) {
    Serial.print("G: ");
    Serial.print(G);
    Serial.print(" Kp: ");
    Serial.print(Kp);
    Serial.print(" Kd: ");
    Serial.print(Kd);
    Serial.print(" Ki: ");
    Serial.println(Ki);
}
