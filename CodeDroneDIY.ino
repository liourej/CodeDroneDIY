#include "CodeDroneDIY.h"

void setup() {
  // Set watchdog reset
  wdt_enable(WDTO_250MS);

  pinMode(12, OUTPUT);

  ESC0.attach(8);
  ESC1.attach(9);
  ESC2.attach(10);
  ESC3.attach(11);
  IdleAllESC();

  InitTimer1();

  // Receiver
  attachInterrupt(0, RxInterrupt, RISING);

  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Wire.setClock(400000L); // Communication with MPU-6050 at 500KHz

  // initialize serial communication
  Serial.begin(250000);

  stateMachine.ArmingSequence();

  // initialize MPU6050 device
  accelgyro.initialize();
  accelgyro.setFullScaleGyroRange( MPU6050_GYRO_FS_1000); //  +-1000°s max  /!\ Be carrefull when changing this parameter: "GyroSensitivity" must be updated accordingly !!!
  accelgyro.setFullScaleAccelRange( MPU6050_ACCEL_FS_8 );//  +-8g max

  if ( !accelgyro.testConnection())
    Serial.println(F("Test failed"));

  Serial.println(F("Computing offsets..."));
  Position.ComputeOffsets(accelgyro);

  while ( !Rx.IsReady() ) {
    IdleAllESC();
    delay(10);
  }

  if ( stateMachine.mode == angle) {
    g_Kp = map(analogRead(2), 0, 1023, 100, 500);
    Serial.println(g_Kp);
    anglePosPIDParams[1] = g_Kp;

    rollPosPID.SetGains(anglePosPIDParams);
    pitchPosPID.SetGains(anglePosPIDParams);

    rollSpeedPID.SetGains(angleSpeedPIDParams);
    pitchSpeedPID.SetGains(angleSpeedPIDParams);
  } else {
    rollSpeedPID.SetGains(accroSpeedPIDParams);
    pitchSpeedPID.SetGains(accroSpeedPIDParams);
  }

  yawSpeedPID.SetGains(yawSpeedPIDParams);

  time.Init();

  PrintSettings(stateMachine);

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

void loop() {
  static float speedCurr[3] = { 0.0, 0.0, 0.0 }; // Teta speed (°/s) (only use gyro)
  static float posCurr[3] = { 0.0, 0.0, 0.0 }; // Teta position (°) (use gyro + accelero)
  static int g_iloop = 0;
  static float g_MeanLoop = 0;
  static int loopNb = 0;
  static float meanLoopTime =  0;
  int throttle = 0;
  float loop_time = time.GetloopTime();
  int rollPosCmd, pitchPosCmd, yawPosCmd = 0;
  int rollMotorPwr, pitchMotorPwr, yawMotorPwr = 0;

  // Get throttle and current position
  throttle = Rx.GetThrottle();

  if ( stateMachine.mode == angle ) {
    Position.GetCurrPos(accelgyro, posCurr, speedCurr, loop_time);
    if ( throttle > 1100 ) {
      stateMachine.throttleWasHigh = true;
      rollPosCmd = rollPosPID.ComputeCorrection( Rx.GetAileronsAngle(), posCurr[0], loop_time );
      rollMotorPwr = rollSpeedPID.ComputeCorrection( rollPosCmd, speedCurr[0], loop_time );

      pitchPosCmd = pitchPosPID.ComputeCorrection( -Rx.GetElevatorAngle(), posCurr[1], loop_time );
      pitchMotorPwr = pitchSpeedPID.ComputeCorrection( -pitchPosCmd, speedCurr[1], loop_time );
      yawMotorPwr = yawSpeedPID.ComputeCorrection( Rx.GetRudder(), speedCurr[2], loop_time );
    } else {

      stateMachine.RefreshState();// Safety cut management: set safety cut after 20 s without power.

      pitchMotorPwr = rollMotorPwr = yawMotorPwr = 0; // No correction if throttle put to min
      rollPosPID.Reset();
      pitchPosPID.Reset();
      rollSpeedPID.Reset();
      pitchSpeedPID.Reset();
      yawSpeedPID.Reset();
    }
  } else { // FLYING_MODE_ACCRO*/
    Position.GetCurrSpeed(accelgyro, speedCurr);
    if ( throttle > 1100 ) {
      stateMachine.throttleWasHigh = true;
      rollMotorPwr = rollSpeedPID.ComputeCorrection( Rx.GetAileronsSpeed(), speedCurr[0], loop_time );
      pitchMotorPwr = pitchSpeedPID.ComputeCorrection( Rx.GetElevatorSpeed(), speedCurr[1], loop_time );
      yawMotorPwr = yawSpeedPID.ComputeCorrection( Rx.GetRudder(), speedCurr[2], loop_time );
    } else {
      stateMachine.RefreshState();// Safety cut management: set safety cut after 5 s without power.

      pitchMotorPwr = rollMotorPwr = yawMotorPwr = 0; // No correction if throttle put to min
      rollSpeedPID.Reset();
      pitchSpeedPID.Reset();
      yawSpeedPID.Reset();
    }
  }

  if ( (stateMachine.mode != disarmed) && (stateMachine.mode != safety) ) {
    //PlusConfig(throttle, pitchMotorPwr, yawMotorPwr, rollMotorPwr);
    XConfig(throttle, pitchMotorPwr, yawMotorPwr, rollMotorPwr);
  }else{
    IdleAllESC();
  }


  if ( loopNb > 1000)
  {
    meanLoopTime = meanLoopTime / loopNb;
    Serial.println(meanLoopTime * 1000, 2);
    meanLoopTime = 0;
    loopNb = 0;
  } else {
    meanLoopTime += loop_time;
    loopNb++;
  }

  wdt_reset();
}

// Notes:
// Inter pos 0: 1900; Inter pos 1: 1496; Inter pos 3: 1088



