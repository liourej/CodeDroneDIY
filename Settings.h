#ifndef SETTINGS_H
#define SETTINGS_H

// Buzzer for lost model alarm
const int BUZZER_PIN  = 7;

// Command setup
const float ALTI_DEAD_ZONE = 0.4; // (%) altimeter dead zone
const float ALTI_LOW_ZONE = 1080+1900*((1-ALTI_DEAD_ZONE)/2); // Upper this limit, vertical speed is positive
const float ALTI_HIGH_ZONE = 1900-1900*((1-ALTI_DEAD_ZONE)/2); // Under this limit, vertical speed is negative
const float ALTI_REFRESH_PERIOD = 95; // (ms)
const unsigned int ALTI_TEMP_REFRESH_PERIOD = 60000; // (ms)
// MAX_THROTTLE set at the top of this file

#endif // not defined SETTINGS_H
