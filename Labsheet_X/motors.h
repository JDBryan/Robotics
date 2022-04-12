// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _MOTORS_H
#define _MOTORS_H

// Class to operate the motor(s).
class Motors_c {
  
  public:
    int l_dir_pin;
    int l_pow_pin;
    int r_dir_pin;
    int r_pow_pin;
    int left_motor_power;
    int right_motor_power;
    float base_speed;
    unsigned long last_update;
    unsigned long update_interval;

    // Constructor, must exist.
    Motors_c() {
    }

    // Use this function to
    // initialise the pins and
    // state of your motor(s).
    void initialise(int ld, int lp, int rd, int rp) {
      l_dir_pin = ld;
      l_pow_pin = lp;
      r_dir_pin = rd;
      r_pow_pin = rp;
      pinMode(l_dir_pin, OUTPUT);
      pinMode(l_pow_pin, OUTPUT);
      pinMode(r_dir_pin, OUTPUT);
      pinMode(r_pow_pin, OUTPUT);
      digitalWrite(l_dir_pin, LOW);
      analogWrite(l_pow_pin, 0);
      digitalWrite(r_dir_pin, LOW);
      analogWrite(r_pow_pin, 0);
      base_speed = 25;
      last_update = millis();
      update_interval = 0;
    }

    void attempt_update(double error) {
      if (millis() - last_update > update_interval) {
        turn(error);
        last_update = millis();
      }
    }

    void set_motors(float l_value, float r_value) {
      set_left_motor(l_value);
      set_right_motor(r_value);
    }

    void turn(double error) {
      error = 2 * error;
      set_left_motor(base_speed - base_speed * max(error, 0));
      set_right_motor(base_speed + base_speed * min(error, 0));
    }

    void set_left_motor(float power) {
      left_motor_power = power;
      set_motor(left_motor_power, l_dir_pin, l_pow_pin);
    }

    void set_right_motor(float power) {
      right_motor_power = power;
      set_motor(right_motor_power, r_dir_pin, r_pow_pin);
    }

    void modify_left_motor(float power) {
      left_motor_power += power;
      set_motor(left_motor_power, l_dir_pin, l_pow_pin);
    }

    void modify_right_motor(float power) {
      right_motor_power += power;
      set_motor(right_motor_power, r_dir_pin, r_pow_pin);
    }
  

  private:
    void set_motor(float power, int dir_pin, int pow_pin) {
      if (power < -100) {
        power = -100;
      } else if (power > 100) {
        power = 100;
      }

      if (power < 0) {
        digitalWrite(dir_pin, HIGH);
      } else {
        digitalWrite(dir_pin, LOW);
      }

      analogWrite(pow_pin, abs(power));
    }
};


#endif
