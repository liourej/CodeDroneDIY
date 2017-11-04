#include <math.h>
#include <avr/wdt.h>
#include "Settings.h"
#include "ESC.h"
#include "Time.h"
#include "Reception.h"
#include "StateMachine.h"
#include "Attitude.h"
#include "PID.h"

Time time;
Time altiTime;
Reception Rx;
ESC ESCs;
PID rollPosPID_Angle, pitchPosPID_Angle, yawPosPID_Angle;
PID rollSpeedPID_Angle, pitchSpeedPID_Angle, yawSpeedPID_Angle, altiSpeedPID_Angle;
PID rollSpeedPID_Accro, pitchSpeedPID_Accro, yawSpeedPID_Accro;
Attitude Attitude;
StateMachine stateMachine;

typedef enum { _timer1, _Nbr_16timers } timer16_Sequence_t;

static inline void handle_interrupts(timer16_Sequence_t timer, volatile uint16_t *TCNTn, volatile uint16_t* OCRnA) {
  ESCs.SetPWM_f5(TCNTn, OCRnA);
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
  if ( _stateMachine.state == angle) {
    Serial.println(F("FLYING_MODE_ANGLE"));
    Serial.println(F("/********* PID settings *********/"));
    rollPosPID_Angle.PrintGains();
    pitchPosPID_Angle.PrintGains();

    rollSpeedPID_Angle.PrintGains();
    pitchSpeedPID_Angle.PrintGains();
    yawSpeedPID_Angle.PrintGains();
    Serial.println(F("/********* Complementary filter *********/"));
    Serial.print("Coefficient: "); Serial.print(Attitude.HighPassFilterCoeff); Serial.print(" Time constant: "); Serial.println(Attitude.GetFilterTimeConstant(0.00249));
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

  Serial.print(F("Mixing: ")); Serial.println(mixing);
  Serial.println(F("/********* Receiver settings *********/"));
  Rx.PrintCmd();
  Serial.println(F("/********* MPU 6050 Configuration *********/"));
//  Serial.print(F("Gyroscope range:\t")); Serial.print(accelgyro.getFullScaleGyroRange()); Serial.print("\tAccelerometer range:\t");  Serial.println(accelgyro.getFullScaleAccelRange());
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

  // Buzzer
  pinMode(BUZZER_PIN, OUTPUT);

  // ESC
  ESCs.Init();

  InitTimer1();

  // Receiver
  attachInterrupt(0, RxInterrupt, RISING); // Receiver interrupt on PD2 (INT0)

  // Console print: initialize serial communication
  //Serial.begin(250000);
  Serial.begin(230400);

  // MPU6050: join I2C bus
  Wire.begin();
  Wire.setClock(400000L); // Communication with MPU-6050 at 400KHz

  // MPU6050, MS5611: initialize MPU6050 and MS5611 devices (IMU and barometer)
  Attitude.Init();

  while ( !Rx.IsReady() ) {
    Serial.println(F("Rx not ready, try again, please wait. "));
   ESCs.Idle();
    wdt_reset();
    delay(200);
  }

  time.InitAllCounters();
  altiTime.InitAllCounters();
  stateMachine.Init();

  // Set watchdog reset
  wdt_enable(WDTO_250MS);

  if ( (MAX_POWER == 1860) && (MAX_THROTTLE >= (1860 * 0.8)) )
    Serial.println(F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!FLYING MODE POWER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! "));
  else if ( (MAX_POWER <= 1300) )
    Serial.println(F("DEBUG MODE POWER!!! "));
  else
    Serial.println(F("UNEXPECTED POWER "));

  Serial.print(F("MAX_POWER: ")); Serial.print(MAX_POWER);  Serial.print(F(" MAX_THROTTLE_PERCENT: ")); Serial.println(MAX_THROTTLE_PERCENT);

  // TODO dirty, but needed for BH_HELI_S, in order to avoid it enter in calibration mode (need to keep ESC idle some seconds):
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
  ESCs.write(ESC0, _throttle - _pitchMotorPwr * mixing + _rollMotorPwr * mixing - _YawMotorPwr * mixing);
  ESCs.write(ESC1,  _throttle - _pitchMotorPwr * mixing - _rollMotorPwr * mixing + _YawMotorPwr * mixing);
  ESCs.write(ESC2,  _throttle + _pitchMotorPwr * mixing - _rollMotorPwr * mixing - _YawMotorPwr * mixing);
  ESCs.write(ESC3,  _throttle + _pitchMotorPwr * mixing + _rollMotorPwr * mixing  + _YawMotorPwr * mixing);
}

void ResetPIDCommand( int *_rollMotorPwr, int *_pitchMotorPwr, int *_yawMotorPwr ) {
  *_pitchMotorPwr = *_rollMotorPwr = *_yawMotorPwr = 0; // No correction if throttle put to min
  rollPosPID_Angle.Reset();
  pitchPosPID_Angle.Reset();
  rollSpeedPID_Angle.Reset();
  pitchSpeedPID_Angle.Reset();
  yawSpeedPID_Angle.Reset();
  rollSpeedPID_Accro.Reset();
  pitchSpeedPID_Accro.Reset();
  yawSpeedPID_Accro.Reset();
  altiSpeedPID_Angle.Reset();
}

void loop() {
  static float speedCurr[3] = { 0.0, 0.0, 0.0 }; // Teta speed (°/s) (only use gyro)
  static float posCurr[3] = { 0.0, 0.0, 0.0 }; // Teta position (°) (use gyro + accelero)
  static int g_iloop = 0;
  static float g_MeanLoop = 0;
  static int loopNb = 0;
  static float meanLoopTime =  0;
  int throttle = 0;
  static float verticalSpeed = 0.0;
  float loopTimeSec = time.GetloopTimeMilliseconds(0);
  int rollPosCmd, pitchPosCmd = 0;
  int rollMotorPwr, pitchMotorPwr, yawMotorPwr = 0;
  static int tempState = disarmed;
  // State Machine
  // initialization -> starting -> angle/accro -> safety -> disarmed -> angle/accro
bool calibrationESC = false;
if( calibrationESC ){
  throttle = Rx.GetThrottle();
  ESCs.write( ESC0, throttle );
  ESCs.write( ESC1, throttle );
  ESCs.write( ESC2, throttle );
  ESCs.write( ESC3, throttle );
  Serial.println(throttle);
  delay(50);
}else{

  switch ( stateMachine.state )
  {
    /*********** ANGLE STATE ***********/
    case angle:
      if ( Attitude.baro_available ) {
        // Compute vertical speed
        if ( altiTime.GetExecutionTimeMilliseconds(0) >= ALTI_REFRESH_PERIOD) {
          verticalSpeed = Attitude.GetVerticalSpeed();
          altiTime.Init(0);
        }
        // refresh temperature for altitude estimation
        if ( altiTime.GetExecutionTimeMilliseconds(1) >= ALTI_TEMP_REFRESH_PERIOD) {
          Attitude.refreshTemperature();
          altiTime.Init(1);
        }
      }

      throttle = Rx.GetThrottle();
      Attitude.GetCurrPos(posCurr, speedCurr, loopTimeSec);
      if ( throttle > IDLE_THRESHOLD ) {
        stateMachine.throttleWasHigh = true;
        rollPosCmd = rollPosPID_Angle.ComputeCorrection( Rx.GetAileronsAngle(), posCurr[0], loopTimeSec );
        rollMotorPwr = rollSpeedPID_Angle.ComputeCorrection( rollPosCmd, speedCurr[0], loopTimeSec );

        pitchPosCmd = pitchPosPID_Angle.ComputeCorrection( Rx.GetElevatorAngle(), posCurr[1], loopTimeSec );
        pitchMotorPwr = pitchSpeedPID_Angle.ComputeCorrection( pitchPosCmd, speedCurr[1], loopTimeSec );

        yawMotorPwr = yawSpeedPID_Angle.ComputeCorrection( Rx.GetRudder(), speedCurr[2], loopTimeSec );

        if ( Attitude.baro_available == true) {
          throttle = altiSpeedPID_Angle.ComputeCorrection( Rx.GetVerticalSpeed(), verticalSpeed, loopTimeSec );
        }

        // Allow to change flying mode during flight
        tempState = Rx.GetFlyingMode();
        if ( tempState == accro ) {
          stateMachine.state = accro;
          Serial.println(F("Flying mode changed from angle to accro"));
        }
      } else {
        stateMachine.RefreshState();// Safety cut management: set safety cut after 20 s without power.
        ResetPIDCommand(&rollMotorPwr, &pitchMotorPwr, &yawMotorPwr);
      }
      XConfig(throttle, pitchMotorPwr, yawMotorPwr, rollMotorPwr);

      break;
    /*********** ACCRO STATE ***********/
    case accro:
      throttle = Rx.GetThrottle();
      Attitude.GetCurrPos( posCurr, speedCurr, loopTimeSec);
      if ( throttle > IDLE_THRESHOLD ) {
        stateMachine.throttleWasHigh = true;
        rollMotorPwr = rollSpeedPID_Accro.ComputeCorrection( Rx.GetAileronsSpeed(), speedCurr[0], loopTimeSec );
        pitchMotorPwr = pitchSpeedPID_Accro.ComputeCorrection( Rx.GetElevatorSpeed(), speedCurr[1], loopTimeSec );
        yawMotorPwr = yawSpeedPID_Accro.ComputeCorrection( Rx.GetRudder(), speedCurr[2], loopTimeSec );
        // Allow to change flying mode during flight
        tempState = Rx.GetFlyingMode();
        if ( tempState == angle ) {
          stateMachine.state = angle;
          Serial.println(F("Flying mode changed from accro to angle"));
        }
      } else {
        stateMachine.RefreshState();// Safety cut management: set safety cut after 5 s without power.
        ResetPIDCommand(&rollMotorPwr, &pitchMotorPwr, &yawMotorPwr);
      }
      XConfig(throttle, pitchMotorPwr, yawMotorPwr, rollMotorPwr);

      break;
    /*********** SAFETY STATE ***********/
    case safety:
      ESCs.Idle();
      stateMachine.state = Rx.GetFlyingMode();
      if ( stateMachine.state != disarmed ) {
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
      if (  stateMachine.state != Rx.GetFlyingMode()) // Check it was not a transitory switch state
        stateMachine.state = disarmed;
      if (stateMachine.state != disarmed) {
        stateMachine.throttleWasHigh = true;
        if ( stateMachine.state == angle)
          Serial.println(F("ANGLE MODE"));
        else if ( stateMachine.state == accro)
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
      if ( stateMachine.state != disarmed )
        stateMachine.state = initialization;
      else if ( Attitude.AreOffsetComputed())
        stateMachine.state =  starting;
      else
        stateMachine.state =  initialization;
      break;
    /*********** STARTING STATE ***********/
    case starting:
      ESCs.Idle();
      stateMachine.state = Rx.GetFlyingMode();
      Pause500ms();
      if (  stateMachine.state != Rx.GetFlyingMode()) // Check it was not a transitory switch state
        stateMachine.state = starting;



      if ( (stateMachine.state == angle) || (stateMachine.state == accro) ) {
        Serial.println(F("stateMachine.state != disarmed MODE"));
        //Angle mode PID config
        rollPosPID_Angle.SetGains(anglePosPIDParams);
        pitchPosPID_Angle.SetGains(anglePosPIDParams);
        rollSpeedPID_Angle.SetGains(angleSpeedPIDParams);
        pitchSpeedPID_Angle.SetGains(angleSpeedPIDParams);

        yawSpeedPIDParams[1] = map(analogRead(0), 0, 1023, 0, 500); // Adjust Kp from potentiometer on A0
        Serial.print("Yaw kP: ");Serial.println(yawSpeedPIDParams[1]);
        yawSpeedPID_Angle.SetGains(yawSpeedPIDParams);

        altiSpeedPIDParams[1] = map(analogRead(2), 0, 1023, 0, 500); // Adjust Kp from potentiometer on A2
        altiSpeedPID_Angle.SetGains(altiSpeedPIDParams);

        //Accro mode PID config
        rollSpeedPID_Accro.SetGains(accroSpeedPIDParams);
        pitchSpeedPID_Accro.SetGains(accroSpeedPIDParams);
        yawSpeedPID_Accro.SetGains(yawSpeedPIDParams);

        PrintSettings(stateMachine);
      } else
        stateMachine.state = starting;

      stateMachine.ActivateBuzzer(500);
      break;
    default:
      Serial.print(F("UNDEFINED STATE!"));
      break;
  }

  // Compute mean loop time and complementary filter time constant
  if ( ((stateMachine.state == angle) || (stateMachine.state == accro)) && ( throttle > IDLE_THRESHOLD )) {
    if ( loopNb > 1000) {
      meanLoopTime = meanLoopTime / loopNb;
      //Serial.println(meanLoopTime, 2);
      //Serial.println(yawSpeedPIDParams[1]);
      //Serial.println(Position.GetFilterTimeConstant(meanLoopTime));
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
