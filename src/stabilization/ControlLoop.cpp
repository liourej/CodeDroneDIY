#include "ControlLoop.h"

void ControlLoop::SetGains(Constants _constants) {
    constants = _constants;
}

void ControlLoop::Reset() {
    errorPrev = 0;
    integrator = 0;
}

int ControlLoop::ComputeCorrection(float _cmd, float _pos, float _loopTime) {
    error = _cmd - _pos;
    integrator = integrator + error;
    int correction =
            (int)(constants.G * (constants.Kp * error + constants.Kd * ((error - errorPrev) / (_loopTime)) + constants.Ki * integrator));

    errorPrev = error;

    // Correction in us
    return correction;
}

void ControlLoop::PrintGains(void) {
    Serial.print("G: ");
    Serial.print(constants.G);
    Serial.print(" Kp: ");
    Serial.print(constants.Kp);
    Serial.print(" Kd: ");
    Serial.print(constants.Kd);
    Serial.print(" Ki: ");
    Serial.println(constants.Ki);
}
