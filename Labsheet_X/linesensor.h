// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _LINESENSOR_H
#define _LINESENSOR_H

// Class to operate the linesensor(s).
class LineSensor_c {
  public:
    int pins[3];
    double readings[3];
    double error;
    unsigned long last_update;
    unsigned long update_interval;
    unsigned long sensor_offset[3];
    unsigned long sensor_range[3];
    
    // Constructor, must exist.
    LineSensor_c() {
    }

    void initialise(int l_pin, int c_pin, int r_pin, int emitter_pin) {
      error = 0;
      update_interval = 0;
      last_update = millis();
      pins[0] = l_pin;
      pins[1] = c_pin;
      pins[2] = r_pin;
      for (int i; i < 3; i++) {
        pinMode(pins[i], INPUT);
      }
      pinMode(emitter_pin, OUTPUT);
      digitalWrite(emitter_pin, HIGH);
    }

    void attempt_update() {
      if (millis() - last_update > update_interval) {
        update_readings();
        adjust_readings();
        calculate_error();
        last_update = millis();
      }
    }

    double calculate_error() {
      double sum = readings[0] + readings[1] + readings[2];
      error = (double)readings[0]/sum - (double)readings[2]/sum;
    }

    void charge_capacitors() {
      for (int i = 0; i < 3; i++) {
        pinMode(pins[i], OUTPUT);
        digitalWrite(pins[i], HIGH);
      }
      
      delayMicroseconds(10);
      
      for (int i = 0; i < 3; i++) {
        pinMode(pins[i], INPUT);
      }
    }

    void calibrate_light() {
      unsigned long left_readings [10];
      unsigned long centre_readings [10];
      unsigned long right_readings [10];

      for (int i; i < 10; i++) {
        update_readings();
        left_readings[i] = readings[0];
        centre_readings[i] = readings[1];
        right_readings[i] = readings[2];
        delay(1000);
      }

      sensor_offset[0] = left_readings[4];
      sensor_offset[1] = centre_readings[4];
      sensor_offset[2] = right_readings[4];
    }

    void calibrate_dark() {
      unsigned long left_readings [10];
      unsigned long centre_readings [10];
      unsigned long right_readings [10];

      for (int i; i < 10; i++) {
        update_readings();
        left_readings[i] = readings[0];
        centre_readings[i] = readings[1];
        right_readings[i] = readings[2];
        delay(1000);
      }

      sensor_range[0] = left_readings[4] - sensor_offset[0];
      sensor_range[1] = centre_readings[4] - sensor_offset[1];
      sensor_range[2] = right_readings[4] - sensor_offset[2];

      Serial.println("Offset");
      Serial.println(sensor_offset[0]);
      Serial.println(sensor_offset[1]);
      Serial.println(sensor_offset[2]);
      Serial.println("");

      Serial.println("Range");
      Serial.println(sensor_range[0]);
      Serial.println(sensor_range[1]);
      Serial.println(sensor_range[2]);
      Serial.println("");
    }

    void adjust_readings() {
      for (int i = 0; i < 3; i++) {
        readings[i] = (double)(readings[i] - sensor_offset[i]) / (double)sensor_range[i];
      }
    }

    bool is_on_line() {
      bool on_line = false;
      for (int i = 0; i < 3; i++) {
        double mid_value = (double)sensor_offset[i] + (double)sensor_range[i]/(double)2.0;
        if (readings[i] > mid_value) on_line = true;
      }
      return on_line;
    }

    void update_readings() {
      int num_sensors = 3;
      int remaining = num_sensors;
      for (int i = 0; i < num_sensors; i++) {
        readings[i] = 0;
      }
      charge_capacitors();
      unsigned long start_time = micros();
      unsigned long timeout = 5000;

      while (remaining > 0) {
        unsigned long elapsed_time = micros() - start_time;
        for (int i = 0; i < num_sensors; i++) {
          if (digitalRead(pins[i]) == LOW && readings[i] == 0) {
            readings[i] = elapsed_time;
            remaining = remaining - 1;
          }
          else if (elapsed_time >= timeout && readings[i] == 0) {
            readings[i] = timeout;
            remaining = remaining - 1;
          }
        }
      }
    }
    
};


#endif
