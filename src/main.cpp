#include <math.h>
#include <avr/wdt.h>
#include "ESC.h"
#include "Time.h"
#include "Reception.h"
#include "StateMachine.h"
#include "Attitude.h"
#include "PID.h"
#include "Stabilization.h"

extern const float GAIN;
extern float anglePosPIDParams[4];
extern float angleSpeedPIDParams[4];
extern float accroSpeedPIDParams[4];
extern float ACCRO_YAW_KP;
extern float yawSpeedPIDParams[4];
const float mixing = 0.5;

Time time;
Reception Rx;
ESC ESCs;
Stabilization Stabilization;
Attitude Attitude;
StateMachine stateMachine;

typedef enum { _timer1, _Nbr_16timers } timer16_Sequence_t;

static inline void handle_interrupts(timer16_Sequence_t timer, volatile uint16_t *TCNTn,
    volatile uint16_t* OCRnA) {
  ESCs.SetPWM_f5(TCNTn, OCRnA);
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
  OCR1A = usToTicks(ESCs.MIN_POWER);

  TIFR1 |= _BV(OCF1A);  // clear any pending interrupts;
  TIMSK1 |=  _BV(OCIE1A);  // enable the output compare interrupt
}

void PrintSettings(StateMachine _stateMachine) {
  Serial.println(F("/********* settings *********/"));
  if (_stateMachine.state == angle) {
    Serial.println(F("FLYING_MODE_ANGLE"));
    Stabilization.PrintAngleModeParameters();
  } else if (_stateMachine.state == accro) {
    Serial.println(F("FLYING_MODE_ACCRO"));
   Stabilization.PrintAccroModeParameters();
     } else if (_stateMachine.state == disarmed) {
    Serial.println(F("DISARMED"));
  } else if (_stateMachine.state == safety) {
    Serial.println(F("SAFETY"));
  }

  Serial.print(F("Mixing: "));
  Serial.println(mixing);
  Serial.println(F("/********* Receiver settings *********/"));
  Rx.PrintCmd();
  Serial.println(F("/********* MPU 6050 Configuration *********/"));
}

// 07/08/2017
// Test 01
// Démarrage des moteurs à 10-20%
// Changements de mode plusieurs fois
// 40 minutes sur le mode accro
// Résultats: oscillations au bout de 40 minutes

void Pause500ms() {
  for (int loop = 0; loop < 5; loop++) {
    delay(100);
    wdt_reset();
  }
}

void setup() {
  // ESC
  ESCs.Init();

  InitTimer1();

  // Receiver
  attachInterrupt(0, RxInterrupt, RISING);  // Receiver interrupt on PD2 (INT0)

  // Console print: initialize serial communication
  // Serial.begin(250000);
  Serial.begin(230400);

  // MPU6050: join I2C bus
  Wire.begin();
  Wire.setClock(400000L);  // Communication with MPU-6050 at 400KHz

  // MPU6050, MS5611: initialize MPU6050 and MS5611 devices (IMU and barometer)
  Attitude.Init();

  while (!Rx.IsReady()) {
    Serial.println(F("Rx not ready, try again, please wait. "));
    ESCs.Idle();
    wdt_reset();
    delay(200);
  }

  time.InitAllCounters();
  stateMachine.Init();

  // Set watchdog reset
  wdt_enable(WDTO_250MS);

  if ((ESCs.MAX_POWER == 1860) && (ESCs.MAX_THROTTLE >= (1860 * 0.8)))
    Serial.println(F("!!!!!!!!!!!!!!!!!!!!FLYING MODE POWER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! "));
  else if ( (ESCs.MAX_POWER <= 1300) )
    Serial.println(F("DEBUG MODE POWER!!! "));
  else
    Serial.println(F("UNEXPECTED POWER "));

  Serial.print(F("MAX_POWER: "));
  Serial.print(ESCs.MAX_POWER);
  Serial.print(F(" MAX_THROTTLE_PERCENT: "));
  Serial.println(ESCs.MAX_THROTTLE_PERCENT);

  // TODO(Julien): dirty, but needed for BH_HELI_S, in order to avoid it enter in calibration mode
  // (need to keep ESC idle some seconds):
  for (int pause = 0; pause < 10; pause++)
    Pause500ms();

  Serial.println(F("Setup Finished"));
}

//    X configuration:
//  ESC0(CCW)  ESC1
//         \  /
//         /  \
//     ESC3   ESC2(CCW)
//
void XConfig(int _throttle, int _pitchMotorPwr, int _YawMotorPwr, int _rollMotorPwr) {
  ESCs.write(ESC0, _throttle - _pitchMotorPwr * mixing
      + _rollMotorPwr * mixing - _YawMotorPwr * mixing);
  ESCs.write(ESC1,  _throttle - _pitchMotorPwr * mixing
      - _rollMotorPwr * mixing + _YawMotorPwr * mixing);
  ESCs.write(ESC2,  _throttle + _pitchMotorPwr * mixing
      - _rollMotorPwr * mixing - _YawMotorPwr * mixing);
  ESCs.write(ESC3,  _throttle + _pitchMotorPwr * mixing
      + _rollMotorPwr * mixing  + _YawMotorPwr * mixing);
}



void loop() {
  static float speedCurr[3] = { 0.0, 0.0, 0.0 };  // Teta speed (°/s) (only use gyro)
  static float posCurr[3] = { 0.0, 0.0, 0.0 };  // Teta position (°) (use gyro + accelero)
  static uint8_t loopNb = 0;
  static float meanLoopTime =  0;
  uint8_t throttle = 0;
  float loopTimeSec = time.GetloopTimeMilliseconds(0);
  int rollPosCmd, pitchPosCmd = 0;
  int rollMotorPwr, pitchMotorPwr, yawMotorPwr = 0;
  static int tempState = disarmed;
  // State Machine
  // initialization -> starting -> angle/accro -> safety -> disarmed -> angle/accro
  bool calibrationESC = false;
  if (calibrationESC) {
    throttle = Rx.GetThrottle();
    ESCs.write(ESC0, throttle);
    ESCs.write(ESC1, throttle);
    ESCs.write(ESC2, throttle);
    ESCs.write(ESC3, throttle);
    Serial.println(throttle);
    delay(50);
  } else {
    switch (stateMachine.state) {
      /*********** ANGLE STATE ***********/
      case angle:
        throttle = Rx.GetThrottle();
        Attitude.GetCurrPos(posCurr, speedCurr, loopTimeSec);
        if (throttle > ESCs.IDLE_THRESHOLD) {
          stateMachine.throttleWasHigh = true;
          Stabilization.Angle(loopTimeSec);

          // Allow to change flying mode during flight
          tempState = Rx.GetFlyingMode();
          if (tempState == accro) {
            stateMachine.state = accro;
            Serial.println(F("Flying mode changed from angle to accro"));
          }
        } else {
          stateMachine.RefreshState();  // Safety cut mngt: set safety cut after 20s without pwr
          Stabilization.ResetPID(&rollMotorPwr, &pitchMotorPwr, &yawMotorPwr);
        }
        XConfig(throttle, pitchMotorPwr, yawMotorPwr, rollMotorPwr);

        break;
        /*********** ACCRO STATE ***********/
      case accro:
        throttle = Rx.GetThrottle();
        Attitude.GetCurrPos(posCurr, speedCurr, loopTimeSec);
        if (throttle > ESCs.IDLE_THRESHOLD) {
          stateMachine.throttleWasHigh = true;
          
          Stabilization.Accro(loopTimeSec);

          // Allow to change flying mode during flight
          tempState = Rx.GetFlyingMode();
          if (tempState == angle) {
            stateMachine.state = angle;
            Serial.println(F("Flying mode changed from accro to angle"));
          }
        } else {
          stateMachine.RefreshState();  // Safety cut management:set safety cut after 5s without pwr
          Stabilization.ResetPID(&rollMotorPwr, &pitchMotorPwr, &yawMotorPwr);
        }
        XConfig(throttle, pitchMotorPwr, yawMotorPwr, rollMotorPwr);

        break;
        /*********** SAFETY STATE ***********/
      case safety:
        ESCs.Idle();
        stateMachine.state = Rx.GetFlyingMode();
        if (stateMachine.state != disarmed) {
          ESCs.Idle();
          stateMachine.state = safety;
        }

        stateMachine.ActivateBuzzer(500);
        break;
        /*********** DISARMED STATE ***********/
      case disarmed:
        ESCs.Idle();
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
        ESCs.Idle();
        while (!Attitude.AreOffsetComputed())
          Attitude.ComputeOffsets();

        stateMachine.state = Rx.GetFlyingMode();
        if (stateMachine.state != disarmed)
          stateMachine.state = initialization;
        else if (Attitude.AreOffsetComputed())
          stateMachine.state =  starting;
        else
          stateMachine.state =  initialization;
        break;
        /*********** STARTING STATE ***********/
      case starting:
        ESCs.Idle();
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
      && (throttle > ESCs.IDLE_THRESHOLD)) {
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
  }
  wdt_reset();
}

// Notes:
// Inter pos 0: 1900; Inter pos 1: 1496; Inter pos 3: 1088
