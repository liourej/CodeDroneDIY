#ifndef PIDCONSTANTS_H_
#define PIDCONSTANTS_H_

class PIDConstants {
  public:
    static PIDConstants *GetInstance() {
        static PIDConstants instance;
        return &instance;
    }

  private:
    PIDConstants(){};
    PIDConstants(PIDConstants const &);   // Don't Implement
    void operator=(PIDConstants const &); // Don't implement

  public:
    // Angle mode
    // These parameters are very important for flight success
    // They must be tuned for each frame type, motors, and propeller used
    // float anglePosPIDParams[4] = {0.010, 268, 0.5, 0.0}; // G, Kp, Kd, Ki
    float anglePosPIDParams[4] = {0.010 /*G*/, 268 /*Kp*/, 0.5 /*Kd*/, 0.0 /*Ki*/};
    float angleSpeedPIDParams[4] = {0.010 /*G*/, 192 /*Kp*/, 0.0 /*Kd*/, 0.0 /*Ki*/};

    // Accro mode
    // 450mm frame, 10x4.5" 2 blades propellers - Tested during flight test: OK
    float accroSpeedPIDParams[4] = {0.010 /*G*/, 192 /*Kp*/, 0.0 /*Kd*/, 0.0 /*Ki*/};

    // Yaw PID
    float yawSpeedPIDParams[4] = {0.010 /*G*/, 150.0 /*Kp*/, 0.0 /*Kd*/, 0.0 /*Ki*/};
};
#endif // PIDCONSTANTS_H_
