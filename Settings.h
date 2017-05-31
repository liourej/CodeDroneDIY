// Power setup
#define MIN_POWER 1060
#define MAX_POWER 1400 // Set to 1860 later to reach max power

// PID setup
#define GAIN 0.010

// Angle mode
#define ANGLE_POS_KP 260 //240
#define ANGLE_POS_KD 0.5 //0.5
#define ANGLE_POS_KI 0.3

#define ANGLE_SPEED_KP 100//160 192
#define ANGLE_SPEED_KD 0.25
#define ANGLE_SPEED_KI 0

#define ANGLE_ROLLPITCH_MAXERROR 60

// Accro mode
#define ACCRO_SPEED_KP 192
#define ACCRO_SPEED_KD 0.25
#define ACCRO_SPEED_KI 0
#define ACCRO_YAW_KP 0 // Not used for now
#define MIXING 0.75

// Command setup
#define MAX_ANGLE  45 // °
#define MAX_ROT_SPEED  135 // °/s
#define MAX_YAW_SPEED  100// °/s
