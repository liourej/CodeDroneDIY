class PID{
private:
  float G = 0;
  float Kp = 0;
  float Kd = 0;
  float Ki = 0;

  float speedCmd = 0;

  float posErrorPrev= 0;
  float posIntegrator = 0;

  float error = 0;
  float errorPrev= 0;
  float integrator = 0;

public:

  void SetPIDCoef(float _G, float _Kp, float _Kd, float _Ki)
  {
    G = _G;
    Kp = _Kp;
    Kd = _Kd;
    Ki = _Ki;    
  };
  void Reset(){ errorPrev = 0; integrator = 0; };

  int GetWaterFallPID( float _posCmd, float _pos, float _speed, float _loopTime )  {
     float posError = 0;
     float correction = 0;
     float speedCmd = 0;

      // Position PID
      posError = _posCmd -_pos;
      if( posError > ANGLE_ROLLPITCH_MAXERROR){
        posError = ANGLE_ROLLPITCH_MAXERROR;
        Serial.println("ANGLE_ROLLPITCH_MAXERROR");
      }    

     posIntegrator = posIntegrator + posError;
     speedCmd = ANGLE_WATERFALL_G*( ANGLE_WATERFALL_KP*posError + ANGLE_WATERFALL_KD*(( posError - posErrorPrev)/(_loopTime)) + ANGLE_WATERFALL_KI*posIntegrator );
     posErrorPrev = posError;

     // Speed PID
     error = speedCmd - _speed;
     integrator = integrator + error;
     correction = G*(Kp*error +  Kd*(( error - errorPrev)/(_loopTime)) + Ki*integrator);
         
     errorPrev = error;

     // Correction in us
     return correction;
     
  };
  
  // Return ESC command in %
 int GetPIDOutput( float _cmd, float _pos, float _loopTime ){
     float correction = 0;
     error = _cmd - _pos;
     integrator = integrator + error;
     correction = G*(Kp*error +  Kd*(( error - errorPrev)/(_loopTime)) + Ki*integrator);
         
     errorPrev = error;

     // Correction in us
     return correction;
   // return (correction * (MAX_POWER - MIN_POWER) / 100);  
  };
};

    /* if( abs(_posCmd)< 2){
        posError = _posCmd -_pos;
        if( posError > ANGLE_ROLLPITCH_MAXERROR){
          posError = ANGLE_ROLLPITCH_MAXERROR;
          Serial.println("ANGLE_ROLLPITCH_MAXERROR");
       }
        error = ANGLE_WATERFALL_KP*posError - _speed;
     }else{
        error = _posCmd - _speed;
     }*/
