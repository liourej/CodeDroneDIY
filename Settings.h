// Power setup
#define MIN_POWER 1060
#define MAX_POWER 1860 // Max power available to stabilize quardirotors - Set to 1860 to reach max power
#define MAX_THROTTLE_PERCENT 80.0
#define MAX_THROTTLE MAX_POWER*(MAX_THROTTLE_PERCENT/100.0) // Threshold on max throttle command (percent of max motor power)
#define IDLE_THRESHOLD 1100

// Reception setup
#define CHANNELS_NB 7

// Buzzer for lost model alarm
#define BUZZER_PIN  8

// PID setup
#define GAIN 0.010

// Angle mode
float anglePosPIDParams[4] = { 0.010, 268, 0.5, 0.0};// G, Kp, Kd, Ki
float angleSpeedPIDParams[4] = { 0.010, 192, 0.0, 0.0};
float altiSpeedPIDParams[4] = { 0.010, 10, 0.0, 0.0};

// Accro mode
//float accroSpeedPIDParams[4] = { 0.010, 192, 0.0, 0.0}; // 450mm frame, 10x4.5" bi-pale - Tested during flight test: OK
float accroSpeedPIDParams[4] = {0.1, 80.0, 30.0, 0.5}; // 220 mm frame, 5x4.5" bi-pale

#define ACCRO_YAW_KP 0 // Not used for now

//Yaw PID
float yawSpeedPIDParams[4] = { 0.010, 150.0, 0.0, 0.0};// G, Kp, Kd, Ki
float mixing = 0.5; //0.75

// Command setup
#define ALTI_DEAD_ZONE 0.4 // (%) altimeter dead zone
#define ALTI_LOW_ZONE 1080+1900*((1-ALTI_DEAD_ZONE)/2) // Upper this limit, vertical speed is positive
#define ALTI_HIGH_ZONE 1900-1900*((1-ALTI_DEAD_ZONE)/2) // Under this limit, vertical speed is negative
#define ALTI_REFRESH_PERIOD 95 // (ms)
unsigned int ALTI_TEMP_REFRESH_PERIOD = 60000; // (ms)
#define ALTI_MAX_VERTICAL_SPEED 2 //(2 m.s-1)
#define MAX_ANGLE  45 // (°) Max roll and pitch angles reachable in angle mode
#define MAX_ROT_SPEED  135 // (°/s) Max roll and pitch speed in accro mode
#define MAX_YAW_SPEED  135// (°/s) Max yaw speed in accro and angle modes
// MAX_THROTTLE set at the top of this file
