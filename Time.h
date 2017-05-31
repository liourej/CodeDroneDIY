class Time {
  private:
    float startTime = 0;
    float prev_time = 0;
    float loop_time = 0; // sec
    bool first_loop = true;

  public:
    void Init()
    {
      startTime = millis();
      prev_time = millis();
    }
    inline float GetloopTime() { // Compute loop time
      loop_time = (millis() - prev_time) / 1000; // Loop time in s
      prev_time = millis();

      return loop_time;
    }

    float GetExecutionTime() {
      return (millis() - startTime) / 1000; // Loop time in s;
    }
};
