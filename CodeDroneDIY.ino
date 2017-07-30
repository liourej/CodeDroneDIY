#include "CodeDroneDIY.h"

void setup() {

  // Buzzer
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);

  // ESC
  ESC0.attach(8);
  ESC1.attach(9);
  ESC2.attach(10);
  ESC3.attach(11);
  IdleAllESC();

  InitTimer1();

  // Receiver
  attachInterrupt(0, RxInterrupt, RISING);

  // Console print: initialize serial communication
  Serial.begin(250000);

  // MPU6050: join I2C bus
  Wire.begin();
  Wire.setClock(400000L); // Communication with MPU-6050 at 400KHz

  // MPU6050: initialize MPU6050 device
  accelgyro.initialize();
  accelgyro.setFullScaleGyroRange( MPU6050_GYRO_FS_1000); //  +-1000°s max  /!\ Be carrefull when changing this parameter: "GyroSensitivity" must be updated accordingly !!!
  accelgyro.setFullScaleAccelRange( MPU6050_ACCEL_FS_8 );//  +-8g max /!\ Be carrefull when changing this parameter: "AcceleroSensitivity" must be updated accordingly !!!
  wdt_reset();
  if ( !accelgyro.testConnection())
    Serial.println(F("Test failed"));

  // IMU check
  Serial.println(F("/********* IMU self-test *********/"));
  if ( !CheckIMU(accelgyro, Position) )
    Serial.println("IMU SELF TEST FAILED !!!!!");
  else
    Serial.println("IMU self test succeed");


  while ( !Rx.IsReady() ) {
    IdleAllESC();
    delay(10);
  }

  time.Init();

  // Set watchdog reset
  wdt_enable(WDTO_250MS);

  if ( MAX_POWER == 1860)
    Serial.println(F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!FLYING MODE POWER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\t"));
  else if ( MAX_POWER <= 1300)
    Serial.println(F("DEBUG MODE POWER!!!\t"));
  Serial.print(F("MAX_POWER:\t")); Serial.println(MAX_POWER);

  Serial.println(F("Setup Finished"));
}

//    + configuration:
//         ESC0(CCW)
//          ^
//          |
//  ESC3 <-- --  ESC1
//          |
//         ESC2(CCW)
//
void PlusConfig(int _throttle, int _pitchMotorPwr, int _YawMotorPwr, int _rollMotorPwr) {
  // Pitch correction
  ESC0.write( _throttle - _pitchMotorPwr - _YawMotorPwr);
  ESC2.write( _throttle + _pitchMotorPwr - _YawMotorPwr);

  // Roll correction
  ESC1.write( _throttle - _rollMotorPwr + _YawMotorPwr);
  ESC3.write( _throttle + _rollMotorPwr + _YawMotorPwr);
}

//    X configuration:
//  ESC0(CCW)  ESC1
//         \  /
//         /  \
//     ESC3   ESC2(CCW)
//
void XConfig(int _throttle, int _pitchMotorPwr, int _YawMotorPwr, int _rollMotorPwr) {
  ESC0.write( _throttle - _pitchMotorPwr * mixing + _rollMotorPwr * mixing - _YawMotorPwr * mixing);
  ESC1.write( _throttle - _pitchMotorPwr * mixing - _rollMotorPwr * mixing + _YawMotorPwr * mixing);
  ESC2.write( _throttle + _pitchMotorPwr * mixing - _rollMotorPwr * mixing - _YawMotorPwr * mixing);
  ESC3.write( _throttle + _pitchMotorPwr * mixing + _rollMotorPwr * mixing  + _YawMotorPwr * mixing);
}

void ResetPIDCommand( int _rollMotorPwr, int _pitchMotorPwr, int _yawMotorPwr ) {
  _pitchMotorPwr = _rollMotorPwr = _yawMotorPwr = 0; // No correction if throttle put to min
  rollPosPID_Angle.Reset();
  pitchPosPID_Angle.Reset();
  rollSpeedPID_Angle.Reset();
  pitchSpeedPID_Angle.Reset();
  yawSpeedPID_Angle.Reset();
  rollSpeedPID_Accro.Reset();
  pitchSpeedPID_Accro.Reset();
  yawSpeedPID_Accro.Reset();
}

void loop() {
  static float speedCurr[3] = { 0.0, 0.0, 0.0 }; // Teta speed (°/s) (only use gyro)
  static float posCurr[3] = { 0.0, 0.0, 0.0 }; // Teta position (°) (use gyro + accelero)
  static int g_iloop = 0;
  static float g_MeanLoop = 0;
  static int loopNb = 0;
  static float meanLoopTime =  0;
  int throttle = 0;
  float loopTimeSec = time.GetloopTime();
  int rollPosCmd, pitchPosCmd, yawPosCmd = 0;
  int rollMotorPwr, pitchMotorPwr, yawMotorPwr = 0;
  static int tempState = disarmed;
  // State Machine
  // initialization -> starting -> angle/accro -> safety -> disarmed -> angle/accro

  switch ( stateMachine.state )
  {
    /*********** ANGLE STATE ***********/
    case angle:
      throttle = Rx.GetThrottle();
      Position.GetCurrPos(accelgyro, posCurr, speedCurr, loopTimeSec);
      if ( throttle > 1100 ) {
        stateMachine.throttleWasHigh = true;
        rollPosCmd = rollPosPID_Angle.ComputeCorrection( Rx.GetAileronsAngle(), posCurr[0], loopTimeSec );
        rollMotorPwr = rollSpeedPID_Angle.ComputeCorrection( rollPosCmd, speedCurr[0], loopTimeSec );

        pitchPosCmd = pitchPosPID_Angle.ComputeCorrection( Rx.GetElevatorAngle(), posCurr[1], loopTimeSec );
        pitchMotorPwr = pitchSpeedPID_Angle.ComputeCorrection( pitchPosCmd, speedCurr[1], loopTimeSec );

        yawMotorPwr = yawSpeedPID_Angle.ComputeCorrection( Rx.GetRudder(), speedCurr[2], loopTimeSec );
      } else {
        stateMachine.RefreshState();// Safety cut management: set safety cut after 20 s without power.
        ResetPIDCommand(rollMotorPwr, pitchMotorPwr, yawMotorPwr);
      }
      XConfig(throttle, pitchMotorPwr, yawMotorPwr, rollMotorPwr);

      // Allow to change flying mode during flight
      tempState = Rx.GetFlyingMode();
      if ( tempState == accro ) {
        stateMachine.state = accro;
        Serial.println("Flying mode changed from angle to accro");
      }
      break;
    /*********** ACCRO STATE ***********/
    case accro:
      throttle = Rx.GetThrottle();
      Position.GetCurrPos(accelgyro, posCurr, speedCurr, loopTimeSec);
      //Position.GetCurrSpeed(accelgyro, speedCurr);
      if ( throttle > 1100 ) {
        stateMachine.throttleWasHigh = true;
        rollMotorPwr = rollSpeedPID_Accro.ComputeCorrection( Rx.GetAileronsSpeed(), speedCurr[0], loopTimeSec );
        pitchMotorPwr = pitchSpeedPID_Accro.ComputeCorrection( Rx.GetElevatorSpeed(), speedCurr[1], loopTimeSec );
        yawMotorPwr = yawSpeedPID_Accro.ComputeCorrection( Rx.GetRudder(), speedCurr[2], loopTimeSec );

      } else {
        stateMachine.RefreshState();// Safety cut management: set safety cut after 5 s without power.
        ResetPIDCommand(rollMotorPwr, pitchMotorPwr, yawMotorPwr);
      }
      XConfig(throttle, pitchMotorPwr, yawMotorPwr, rollMotorPwr);

      // Allow to change flying mode during flight
      tempState = Rx.GetFlyingMode();
      if ( tempState == angle ) {
        stateMachine.state = angle;
        Serial.println("Flying mode changed from accro to angle");
      }
      break;
    /*********** SAFETY STATE ***********/
    case safety:
      stateMachine.state = Rx.GetFlyingMode();
      if ( stateMachine.state != disarmed ) {
        IdleAllESC();
        stateMachine.state = safety;
      }

      if ( Rx.GetSwitchH() )
        ActivateBuzzer(0.005, 500);
      break;
    /*********** DISARMED STATE ***********/
    case disarmed:
      stateMachine.state = Rx.GetFlyingMode();
      delay(200);
      wdt_reset();
      delay(200);
      wdt_reset();
      if (  stateMachine.state != Rx.GetFlyingMode()) // Check it was not a transitory switch state
        stateMachine.state = disarmed;
      /* if ( (stateMachine.state != disarmed) &&
            (stateMachine.state != stateMachine.statePrev) ) {
         stateMachine.state  = safety;
         Serial.println(F("Choose same state than previous used"));
         IdleAllESC();
        } else*/
      if (stateMachine.state != disarmed) {
        stateMachine.statePrev = stateMachine.state;
        stateMachine.throttleWasHigh = true;
        if ( stateMachine.state == angle)
          Serial.println("ANGLE MODE");
        else if ( stateMachine.state == accro)
          Serial.println("ACCRO MODE");
      }

      if ( Rx.GetSwitchH() )
        ActivateBuzzer(0.005, 500);
      break;
    /*********** INITIALIZATION STATE ***********/
    case initialization:
      IdleAllESC();
      while (!Position.AreOffsetComputed())
        Position.ComputeOffsets(accelgyro);

      stateMachine.state = Rx.GetFlyingMode();
      if ( stateMachine.state != disarmed )
        stateMachine.state = initialization;
      else if ( Position.AreOffsetComputed())
        stateMachine.state =  starting;
      if ( Rx.GetSwitchH() )
        ActivateBuzzer(0.005, 500);
      break;
    /*********** STARTING STATE ***********/
    case starting:
      Serial.println("stateMachine.state starting");
      IdleAllESC();
      stateMachine.state = Rx.GetFlyingMode();
      delay(200);
      wdt_reset();
      delay(200);
      wdt_reset();
      if (  stateMachine.state != Rx.GetFlyingMode()) // Check it was not a transitory switch state
        stateMachine.state = starting;

      if ( (stateMachine.state == angle) || (stateMachine.state == accro) ) {
        Serial.println("stateMachine.state != disarmed MODE");
        //Angle mode PID config
        // anglePosPIDParams[1] = map(analogRead(2), 0, 1023, 100, 500); // Adjust Kp from potentiometer
        //anglePosPIDParams[3] = map(analogRead(3), 0, 1023, 0, 100); // Adjust Ki from potentiometer
        rollPosPID_Angle.SetGains(anglePosPIDParams);
        pitchPosPID_Angle.SetGains(anglePosPIDParams);
        rollSpeedPID_Angle.SetGains(angleSpeedPIDParams);
        pitchSpeedPID_Angle.SetGains(angleSpeedPIDParams);
        yawSpeedPID_Angle.SetGains(yawSpeedPIDParams);

        //Accro mode PID config
        rollSpeedPID_Accro.SetGains(accroSpeedPIDParams);
        pitchSpeedPID_Accro.SetGains(accroSpeedPIDParams);
        yawSpeedPID_Accro.SetGains(yawSpeedPIDParams);

        stateMachine.statePrev = stateMachine.state;
        PrintSettings(stateMachine);

      } else
        stateMachine.state = starting;

      if ( Rx.GetSwitchH() )
        ActivateBuzzer(0.005, 500);
      break;
    default:
      Serial.print("UNDEFINED STATE!");
      break;
  }

  // Compute mean loop time and complementary filter time constant
  if ( ((stateMachine.state == angle) || (stateMachine.state == accro)) && ( throttle > 1100 )) {
    if ( loopNb > 1000) {
      meanLoopTime = meanLoopTime / loopNb;
      // Serial.println(meanLoopTime * 1000, 2);
      //Serial.println(Position.GetFilterTimeConstant(meanLoopTime));
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



