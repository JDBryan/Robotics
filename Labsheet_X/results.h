
#define NUM_RESULTS 140

class Results_c {
  private:
    int result_index = 0;
    float powers[NUM_RESULTS];
    float velocities[NUM_RESULTS];
    unsigned int times[NUM_RESULTS];
    unsigned long last_print;

    unsigned long last_update = 0;
    unsigned long time_interval = 100;
  public:
    void attempt_update(double power, double velocity) {
      if (time_interval < (millis() - last_update)) {
        log_result(power, velocity);
        last_update = millis();
      }
    }

  
    void log_result(double power, double velocity) {
      if (result_index >= NUM_RESULTS) {
          analogWrite(6, 30);
          return;
      }
      powers[result_index] = power;
      velocities[result_index] = velocity;
      times[result_index] = millis();
      result_index += 1; 
    }

    void print_results() {
      
      if (millis() - last_print < 1000) {
        return;
      }
      
      Serial.println("time, power, velocity");
      for (int i = 0; i < result_index; i++) {
        Serial.print(times[i]);
        Serial.print(",");
        Serial.print(powers[i]);
        Serial.print(",");
        Serial.print(velocities[i]);
        Serial.println("");
      }
      last_print = millis();
    }
};
