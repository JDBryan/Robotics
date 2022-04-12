// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _PID_H
#define _PID_H

// Class to contain generic PID algorithm.
class PID_c {
  public:
    double delta_time;
    double p_term;
    double i_term;
    double d_term;
    double k_p;
    double k_i;
    double k_d;
    double previous_error;
    double feedback_signal;
    unsigned long last_update;
    unsigned long update_interval;
    
    // Constructor, must exist.
    PID_c() {
      
    }

    void initialise(double p_gain, double i_gain, double d_gain) { 
      k_p = p_gain;
      k_i = i_gain;
      k_d = d_gain;
      i_term = 0;
      previous_error = 0;
      feedback_signal = 0;
      last_update = millis();
      update_interval = 30;
    }

    void attempt_update(float demand, float measurement) {
      if (millis() - last_update > update_interval) {
        update(demand, measurement);
        last_update = millis();
      }
    }

    void reset() {
      i_term = 0;
      last_update = millis();
    }

    void update(double demand, double measurement) {
      double error = demand - measurement;
      unsigned long delta_time = millis() - last_update;
      p_term = k_p * error;
      i_term += k_i * error * delta_time;
      d_term = k_d * ((error - previous_error)/delta_time);
      feedback_signal = p_term + i_term + d_term;
      previous_error = error;
    }
};



#endif
