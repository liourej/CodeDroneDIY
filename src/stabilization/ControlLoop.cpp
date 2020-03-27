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
    CustomSerialPrint::print("G: ");
    CustomSerialPrint::print(constants.G);
    CustomSerialPrint::print(" Kp: ");
    CustomSerialPrint::print(constants.Kp);
    CustomSerialPrint::print(" Kd: ");
    CustomSerialPrint::print(constants.Kd);
    CustomSerialPrint::print(" Ki: ");
    CustomSerialPrint::println(constants.Ki);
}
