class PID{
private:
  float G = 0;
  float Kp = 0;
  float Kd = 0;
  float Ki = 0;

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
  // Return ESC command in %
  int GetPIDOutput( const float _cmd, const float _pos, const float _loopTime ){
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
