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

Time time, safetyCut;
bool firstTime = true;
MPU6050 accelgyro;
Reception Rx;
PID rollPosPID, pitchPosPID, yawPosPID;
PID rollSpeedPID, pitchSpeedPID, yawSpeedPID;
GetPosition Position;
int g_FlyingMode = disarmed;
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

void ArmingSequence() {
  g_FlyingMode = Rx.GetFlyingMode();
  if ( g_FlyingMode != disarmed )
    Serial.println("Disarm before continue");

  while (g_FlyingMode != disarmed ) {
    ESC0.Idle();
    ESC1.Idle();
    ESC2.Idle();
    ESC3.Idle();
    g_FlyingMode = Rx.GetFlyingMode();
    delay(200);
    wdt_reset();
  }

  Serial.println("Select flying mode");

  while (g_FlyingMode == disarmed) {
    // Wait 2 sec to let user select the mode using the 3 positions switch
    for (int i = 0; i < 10; i++) {
      delay(200);
      wdt_reset();
    }
    g_FlyingMode = Rx.GetFlyingMode();
    if ( !firstTime && g_FlyingMode != Rx.GetFlyingModePrev() )
    {
      g_FlyingMode  = disarmed;
      Serial.println("Choose same mode than previous used");
    }
    ESC0.Idle();
    ESC1.Idle();
    ESC2.Idle();
    ESC3.Idle();
    wdt_reset();
  }
  Rx.SetFlyingModePrev(g_FlyingMode);
}

void PrintSettings(void) {
  Serial.print("MAX_POWER:\t"); Serial.println(MAX_POWER);
  if ( g_FlyingMode == FLYING_MODE_ANGLE) {
    Serial.println("FLYING_MODE_ANGLE");
    Serial.println("/********* PID settings *********/");
    rollPosPID.PrintGains();
    pitchPosPID.PrintGains();

    rollSpeedPID.PrintGains();
    pitchSpeedPID.PrintGains();
    yawSpeedPID.PrintGains();
  } else {
    Serial.println("FLYING_MODE_ACCRO");
    Serial.println("/********* PID settings *********/");
    rollSpeedPID.PrintGains();
    pitchSpeedPID.PrintGains();
    yawSpeedPID.PrintGains();
  }
  Serial.print("Mixing:\t"); Serial.println(mixing);
  Serial.println("/********* Receiver settings *********/");
  Rx.PrintCmd();
  Serial.println("/********* MPU 6050 Configuration *********/");
  Serial.print("Gyroscope range:\t"); Serial.print(accelgyro.getFullScaleGyroRange()); Serial.print("\tAccelerometer range:\t");  Serial.println(accelgyro.getFullScaleAccelRange());
}
