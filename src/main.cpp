#include <math.h>
#include <avr/wdt.h>
#include "Time.h"
#include "Reception.h"
#include "StateMachine.h"
#include "Stabilization.h"

extern const float GAIN;
extern float ACCRO_YAW_KP;
extern float yawSpeedPIDParams[4];

Time time;
Reception Rx;
Stabilization stabilization;
Attitude attitude;
StateMachine stateMachine;

typedef enum { _timer1, _Nbr_16timers } timer16_Sequence_t;

static inline void handle_interrupts(timer16_Sequence_t timer, volatile uint16_t *TCNTn,
    volatile uint16_t* OCRnA) {
  stabilization.SetESCsPWM(TCNTn, OCRnA);
}

SIGNAL(TIMER1_COMPA_vect) {
  handle_interrupts(_timer1, &TCNT1, &OCR1A);
}

void RxInterrupt() {
  Rx.GetWidth();
}

void InitTimer1() {
  // Timer
  TCCR1A = 0;  // normal counting mode
  TCCR1B = _BV(CS10);  // no prescaler
  TCNT1 = 0;  // clear the timer count
  OCR1A = usToTicks(stabilization.GetESCsMinPower());

  TIFR1 |= _BV(OCF1A);  // clear any pending interrupts;
  TIMSK1 |=  _BV(OCIE1A);  // enable the output compare interrupt
}

void PrintSettings(StateMachine _stateMachine) {
  Serial.println(F("/********* settings *********/"));
  if (_stateMachine.state == angle) {
    Serial.println(F("FLYING_MODE_ANGLE"));
    stabilization.PrintAngleModeParameters();
  } else if (_stateMachine.state == accro) {
    Serial.println(F("FLYING_MODE_ACCRO"));
    stabilization.PrintAccroModeParameters();
  } else if (_stateMachine.state == disarmed) {
    Serial.println(F("DISARMED"));
  } else if (_stateMachine.state == safety) {
    Serial.println(F("SAFETY"));
  }

  Serial.println(F("/********* Receiver settings *********/"));
  Rx.PrintCmd();
  Serial.println(F("/********* MPU 6050 Configuration *********/"));
}

void Pause500ms() {
  for (int loop = 0; loop < 5; loop++) {
    delay(100);
    wdt_reset();
  }
}

void setup() {
  InitTimer1();

  // Receiver
  attachInterrupt(0, RxInterrupt, RISING);  // Receiver interrupt on PD2 (INT0)

  // Console print: initialize serial communication
  // Serial.begin(250000);
  Serial.begin(230400);

  stabilization.Init(Rx);

  time.InitAllCounters();
  stateMachine.Init();

  // Set watchdog reset
  wdt_enable(WDTO_250MS);

  if ((stabilization.GetESCsMaxPower() == 1860) && (stabilization.GetESCsMaxThrottle() >= (1860 * 0.8)))
    Serial.println(F("!!!!!!!!!!!!!!!!!!!!FLYING MODE POWER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! "));
  else if ( (stabilization.GetESCsMaxPower() <= 1300) )
    Serial.println(F("DEBUG MODE POWER!!! "));
  else
    Serial.println(F("UNEXPECTED POWER "));

  Serial.print(F("MAX_POWER: "));
  Serial.print(stabilization.GetESCsMaxPower());
  Serial.print(F(" MAX_THROTTLE_PERCENT: "));
  Serial.println(stabilization.GetESCsMaxThrottlePercent());

  Serial.println(F("Setup Finished"));
}



void loop() {
  static uint8_t loopNb = 0;
  static float meanLoopTime =  0;
  uint8_t throttle = 0;
  float loopTimeSec = time.GetloopTimeMilliseconds(0);
  static int tempState = disarmed;
  // State Machine
  // initialization -> starting -> angle/accro -> safety -> disarmed -> angle/accro
  switch (stateMachine.state) {
    /*********** ANGLE STATE ***********/
    case angle:
      throttle = Rx.GetThrottle(stabilization.GetESCsMinPower(), stabilization.GetESCsMaxThrottle());
      if (throttle > stabilization.GetESCIdleThreshold()) {
        stateMachine.throttleWasHigh = true;
        stabilization.Angle(loopTimeSec, Rx);

        // Allow to change flying mode during flight
        tempState = Rx.GetFlyingMode();
        if (tempState == accro) {
          stateMachine.state = accro;
          Serial.println(F("Flying mode changed from angle to accro"));
        }
      } else {
        stateMachine.RefreshState();  // Safety cut mngt: set safety cut after 20s without pwr
        stabilization.ResetPID();
      }
      stabilization.XConfig(throttle);

      break;
      /*********** ACCRO STATE ***********/
    case accro:
      throttle = Rx.GetThrottle(stabilization.GetESCsMinPower(), stabilization.GetESCsMaxThrottle());
      if (throttle > stabilization.GetESCIdleThreshold()) {
        stateMachine.throttleWasHigh = true;

        stabilization.Accro(loopTimeSec, Rx);

        // Allow to change flying mode during flight
        tempState = Rx.GetFlyingMode();
        if (tempState == angle) {
          stateMachine.state = angle;
          Serial.println(F("Flying mode changed from accro to angle"));
        }
      } else {
        stateMachine.RefreshState();  // Safety cut management:set safety cut after 5s without pwr
        stabilization.ResetPID();
      }
      stabilization.XConfig(throttle);

      break;
      /*********** SAFETY STATE ***********/
    case safety:
      stabilization.Idle();
      stateMachine.state = Rx.GetFlyingMode();
      if (stateMachine.state != disarmed) {
        stabilization.Idle();
        stateMachine.state = safety;
      }

      stateMachine.ActivateBuzzer(500);
      break;
      /*********** DISARMED STATE ***********/
    case disarmed:
      stabilization.Idle();
      stateMachine.state = Rx.GetFlyingMode();
      Pause500ms();
      // Check it was not a transitory switch state
      if (stateMachine.state != Rx.GetFlyingMode())
        stateMachine.state = disarmed;
      if (stateMachine.state != disarmed) {
        stateMachine.throttleWasHigh = true;
        if (stateMachine.state == angle)
          Serial.println(F("ANGLE MODE"));
        else if (stateMachine.state == accro)
          Serial.println(F("ACCRO MODE"));
        else
          stateMachine.state = disarmed;
      }

      stateMachine.ActivateBuzzer(500);
      break;
      /*********** INITIALIZATION STATE ***********/
    case initialization:
      stabilization.Idle();
      while (!attitude.AreOffsetComputed())
        attitude.ComputeOffsets();

      stateMachine.state = Rx.GetFlyingMode();
      if (stateMachine.state != disarmed)
        stateMachine.state = initialization;
      else if (attitude.AreOffsetComputed())
        stateMachine.state =  starting;
      else
        stateMachine.state =  initialization;
      break;
      /*********** STARTING STATE ***********/
    case starting:
      stabilization.Idle();
      stateMachine.state = Rx.GetFlyingMode();
      Pause500ms();
      if (stateMachine.state != Rx.GetFlyingMode())  // Check it was not a transitory switch state
        stateMachine.state = starting;
      if ((stateMachine.state == angle) || (stateMachine.state == accro)) {
        Serial.println(F("stateMachine.state != disarmed MODE"));

        PrintSettings(stateMachine);
      } else {
        stateMachine.state = starting;
      }

      stateMachine.ActivateBuzzer(500);
      break;
    default:
      Serial.print(F("UNDEFINED STATE!"));
      break;
  }

  // Compute mean loop time and complementary filter time constant
  if (((stateMachine.state == angle) || (stateMachine.state == accro))
      && (throttle > stabilization.GetESCIdleThreshold())) {
    if (loopNb > 1000) {
      meanLoopTime = meanLoopTime / loopNb;
      Serial.println(meanLoopTime, 2);
      meanLoopTime = 0;
      loopNb = 0;
    } else {
      meanLoopTime += loopTimeSec;
      loopNb++;
    }
  }
  wdt_reset();
}

// Notes:
// Inter pos 0: 1900; Inter pos 1: 1496; Inter pos 3: 1088
