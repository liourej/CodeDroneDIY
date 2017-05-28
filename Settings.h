// Power setup
#define MIN_POWER 1060
#define MAX_POWER 1400 // Set to 1860 later to reach max power

// PID setup
// Angle mode
#define ANGLE_GAIN 0.0012
#define ANGLE_ROLLPITCH_KP 90
#define ANGLE_ROLLPITCH_KD 60
#define ANGLE_ROLLPITCH_KI 0.05

// Accro mode
#define ACCRO_GAIN 0.010
#define ACCRO_ROLLPITCH_KP 192
#define ACCRO_ROLLPITCH_KD 0.25
#define ACCRO_ROLLPITCH_KI 0
#define ACCRO_YAW_KP 0 // Not used for now
#define MIXING 0.75

// Command setup
#define MAX_ANGLE  30 // °
#define MAX_ROT_SPEED  135 // °/s
#define MAX_YAW_SPEED  100// °/s
