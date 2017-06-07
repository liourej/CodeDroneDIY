// Power setup
#define MIN_POWER 1060
#define MAX_POWER 1400 // Set to 1860 later to reach max power

// PID setup
#define GAIN 0.010

// Angle mode
float anglePosPIDParams[4] = { 0.010, 260, 0.5, 0.3};
float angleSpeedPIDParams[4] = { 0.010, 100, 0.25, 0.0};

// Accro mode
float accroSpeedPIDParams[4] = { 0.010, 192, 0.25, 0.0};
#define ACCRO_YAW_KP 0 // Not used for now

//Yaw PID
float yawSpeedPIDParams[4] = { 0.010, 0.0, 0.0, 0.0};// G, Kp, Kd, Ki
#define MIXING 1 //0.75

// Command setup
#define MAX_ANGLE  45 // °
#define MAX_ROT_SPEED  135 // °/s
#define MAX_YAW_SPEED  100// °/s
