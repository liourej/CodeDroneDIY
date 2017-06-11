#include <math.h>
#include <avr/wdt.h>
#include "Settings.h"
#include "GetPosition.h"
#include "ESC.h"
#include "Reception.h"
#include "Time.h"
#include "PID.h"
#include "SetPWM.h"

typedef enum { _timer1, _Nbr_16timers } timer16_Sequence_t;

Time time;
MPU6050 accelgyro;
Reception Rx;
PID rollPosPID, pitchPosPID, yawPosPID;
PID rollSpeedPID, pitchSpeedPID, yawSpeedPID;
GetPosition Position;
int g_FlyingMode = FLYING_MODE_ACCRO;
int g_Kp = 0;
bool g_YawPIDActivated = false;
float speedCurr[3] = { 0.0, 0.0, 0.0 }; // Teta speed (°/s) (only use gyro)
float posCurr[3] = { 0.0, 0.0, 0.0 }; // Teta position (°) (use gyro + accelero)
int g_iloop = 0;
float g_MeanLoop = 0;
int loopNb = 0;
float meanLoopTime =  0;

void idleESC() {
  ESC0.write(MIN_POWER);
  ESC1.write(MIN_POWER);
  ESC2.write(MIN_POWER);
  ESC3.write(MIN_POWER);
  ESCList[0] = ESC0;
  ESCList[1] = ESC1;
  ESCList[2] = ESC2;
  ESCList[3] = ESC3;
  g_NewVal = true;
}

static inline void handle_interrupts(timer16_Sequence_t timer, volatile uint16_t *TCNTn, volatile uint16_t* OCRnA) {
  // SetPWM_f4(TCNTn, OCRnA);
  SetPWM_f5(TCNTn, OCRnA);
}

SIGNAL (TIMER1_COMPA_vect) {
  handle_interrupts(_timer1, &TCNT1, &OCR1A);
}

void RxInterrupt() {
  Rx.GetWidth();
}

void InitTimer1() {
  // Timer
  TCCR1A = 0;             // normal counting mode
  TCCR1B = _BV(CS10);     // no prescaler
  TCNT1 = 0;              // clear the timer count
  OCR1A = usToTicks(MIN_POWER);

  TIFR1 |= _BV(OCF1A);      // clear any pending interrupts;
  TIMSK1 |=  _BV(OCIE1A) ;  // enable the output compare interrupt
}

void PrintSettings(void) {
  Serial.print("MAX_POWER:\t"); Serial.println(MAX_POWER);
  if ( g_FlyingMode == FLYING_MODE_ANGLE) {
    Serial.println("FLYING_MODE_ANGLE");
    rollPosPID.PrintGains();
    pitchPosPID.PrintGains();

    rollSpeedPID.PrintGains();
    pitchSpeedPID.PrintGains();
    yawSpeedPID.PrintGains();
  } else {
    Serial.println("FLYING_MODE_ACCRO");
    rollSpeedPID.PrintGains();
    pitchSpeedPID.PrintGains();
    yawSpeedPID.PrintGains();
  }
  Rx.PrintCmd();
  Serial.print("Yaw PID activation:\t"); Serial.println(g_YawPIDActivated);
  Serial.print("Mixing:\t"); Serial.println(mixing);
}
