#include <math.h>
#include <avr/wdt.h>
#include "Settings.h"
#include "GetPosition.h"
#include "ESC.h"
#include "Reception.h"
#include "Time.h"
#include "PID.h"
#include "SetPWM.h"
#include "StateMachine.h"
#include "checkIMU.h"

typedef enum { _timer1, _Nbr_16timers } timer16_Sequence_t;

Time time;

Reception Rx;

PID rollPosPID_Angle, pitchPosPID_Angle, yawPosPID_Angle;
PID rollSpeedPID_Angle, pitchSpeedPID_Angle, yawSpeedPID_Angle;
PID rollSpeedPID_Accro, pitchSpeedPID_Accro, yawSpeedPID_Accro;
GetPosition Position;
MPU6050 accelgyro;
StateMachine stateMachine;

void ActivateBuzzer(float _frequency, int _duration) {
  Time time;
  time.Init();
  while ( (time.GetExecutionTime() * 1000) < _duration) {
    digitalWrite(12, HIGH);
    delay(1 / (2 * _frequency) );
    digitalWrite(12, LOW);
    delay(1 / (2 * _frequency) );
    wdt_reset();
    Serial.println(F("BUZZZZZ"));
  }
}

void IdleAllESC() {
  ESC0.Idle();
  ESC1.Idle();
  ESC2.Idle();
  ESC3.Idle();
}

static inline void handle_interrupts(timer16_Sequence_t timer, volatile uint16_t *TCNTn, volatile uint16_t* OCRnA) {
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

void PrintSettings(StateMachine _stateMachine) {
  Serial.println(F("/********* settings *********/"));
  if ( MAX_POWER == 1860)
    Serial.println(F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!FLYING MODE POWER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\t"));
  else if ( MAX_POWER <= 1300)
    Serial.println(F("DEBUG MODE POWER!!!\t"));
  Serial.print(F("MAX_POWER:\t")); Serial.println(MAX_POWER);
  if ( _stateMachine.state == angle) {
    Serial.println(F("FLYING_MODE_ANGLE"));
    Serial.println(F("/********* PID settings *********/"));
    rollPosPID_Angle.PrintGains();
    pitchPosPID_Angle.PrintGains();

    rollSpeedPID_Angle.PrintGains();
    pitchSpeedPID_Angle.PrintGains();
    yawSpeedPID_Angle.PrintGains();
    Serial.println(F("/********* Complementary filter *********/"));
    Serial.print("Coefficient:\t"); Serial.print(Position.HighPassFilterCoeff); Serial.print("\tTime constant:\t"); Serial.println(Position.GetFilterTimeConstant(0.00249));
  } else if ( _stateMachine.state == accro) {
    Serial.println(F("FLYING_MODE_ACCRO"));
    Serial.println(F("/********* PID settings *********/"));
    rollSpeedPID_Accro.PrintGains();
    pitchSpeedPID_Accro.PrintGains();
    yawSpeedPID_Accro.PrintGains();
  } else if ( _stateMachine.state == disarmed) {
    Serial.println(F("DISARMED"));
  } else if ( _stateMachine.state == safety) {
    Serial.println(F("SAFETY"));
  }

  Serial.print(F("Mixing:\t")); Serial.println(mixing);
  Serial.println(F("/********* Receiver settings *********/"));
  Rx.PrintCmd();
  Serial.println(F("/********* MPU 6050 Configuration *********/"));
  Serial.print(F("Gyroscope range:\t")); Serial.print(accelgyro.getFullScaleGyroRange()); Serial.print("\tAccelerometer range:\t");  Serial.println(accelgyro.getFullScaleAccelRange());
}
