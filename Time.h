class Time {
  private:
    float startTime[2] = {0.0, 0.0};
    float prev_time[2] = {0.0, 0.0};
    float loop_time[2] = {0.0, 0.0}; // sec
    bool first_loop[2] = {true, true};

  public:
    void InitAllCounters(){
      for (int i = 0; i < 2; i++) {
        startTime[i] = millis();
        prev_time[i] = millis();
      }
    }

    void Init(int _counter){
      if ( _counter > 2 )
        return;
        
      startTime[_counter] = millis();
      prev_time[_counter] = millis();
    }

    inline float GetloopTime(int _counter) { // Compute loop time
      if ( _counter > 2 )
        return -1;
        
      loop_time[_counter] = (millis() - prev_time[_counter]) / 1000; // Loop time in s
      prev_time[_counter] = millis();

      return loop_time[_counter];
    }

    float GetExecutionTime(int _counter) {
      if ( _counter > 2 )
        return -1;
        
      return (millis() - startTime[_counter])/1000; // Loop time in s;
    }
};
