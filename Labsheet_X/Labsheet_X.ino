#include "motors.h"
#include "linesensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "pid.h"
#include "results.h"

#define SENSOR_LEFT_PIN A0
#define SENSOR_CENTRE_PIN A2
#define SENSOR_RIGHT_PIN A3
#define IR_EMITTER 11

#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN 9
#define R_DIR_PIN 15

#define START 0
#define FIND_LINE 1
#define FOLLOW_LINE 2

int state;
double loop_interval;
double e0_change;
double e1_change;
double l_wheel_velocity;
double r_wheel_velocity;
double last_update = micros();
double last_e0;
double last_e1;
Motors_c motors;
PID_c r_wheel_pid;
PID_c l_wheel_pid;
PID_c line_following_pid;
LineSensor_c line_sensors;
Kinematics_c kinematics;
Results_c results;



void oldsetup() {
  Serial.begin(9600);
  //  delay(10000);
  setupEncoder0();
  setupEncoder1();
  Serial.println("***RESET***");
  
//  Serial.println(count_e0)
  state = START;
}

void initialise() {
  Serial.println("INITIALISING");
  motors.initialise(L_DIR_PIN, L_PWM_PIN, R_DIR_PIN, R_PWM_PIN);
  line_sensors.initialise(SENSOR_LEFT_PIN, SENSOR_CENTRE_PIN, SENSOR_RIGHT_PIN, IR_EMITTER);
  r_wheel_pid.initialise(50, 0.05, 5);
  l_wheel_pid.initialise(50, 0.05, 5);
  line_following_pid.initialise(1, 0.1, 0.1);
  pinMode(17, INPUT);
  pinMode(30, INPUT);
//  Serial.println(count_e0);
}

void calibrate() {
  line_sensors.calibrate_light();
  motors.set_motors(20, -20);
  delay(5000);
  motors.set_motors(0, 0);
  delay(5000);
  line_sensors.calibrate_dark();
}

void loopold() {
  
  if (state == START) {
    Serial.println("Started Calibrating");
    initialise();
//    calibrate();
    state = FOLLOW_LINE;
    r_wheel_pid.reset();
    l_wheel_pid.reset();
    Serial.println("Done Calibrating");
  } else if (state == FOLLOW_LINE) {
//    line_sensors.attempt_update();
//    line_following_pid.attempt_update(0, line_sensors.error);
//    double base_velocity = 0.5;
//    double l_target_velocity = base_velocity - base_velocity * max(line_following_pid.feedback_signal, 0);
//    double r_target_velocity = base_velocity + base_velocity * min(line_following_pid.feedback_signal, 0);
    r_wheel_pid.attempt_update(0.5, r_wheel_velocity);
    l_wheel_pid.attempt_update(0.5, l_wheel_velocity);
    motors.set_right_motor(r_wheel_pid.feedback_signal);
    motors.set_left_motor(l_wheel_pid.feedback_signal);
    
    kinematics.attempt_update();
    results.attempt_update(r_wheel_pid.feedback_signal, r_wheel_velocity);
    
    if (digitalRead(17) == LOW) {
      results.print_results();
    }
  }

  if (micros() - last_update > 200000) {
      e0_change = count_e0 - last_e0;
      e1_change = count_e1 - last_e1;
      loop_interval = micros() - last_update;
      last_update = micros();
      last_e0 = count_e0;
      last_e1 = count_e1;
      r_wheel_velocity = -1000 * (e0_change / loop_interval);
      l_wheel_velocity = -1000 * (e1_change / loop_interval);
   }
}

enum JState {
  WAITING,
  RUNNING,
  TRANSITION,
  RECORDING,
  STOPPED,
};
JState jstate = WAITING;

void setup() {
  initialise();
  setupEncoder0();
  setupEncoder1();
}

void loop() {
  if (jstate == WAITING) {
    if (digitalRead(17) == LOW) {
      jstate = RUNNING;
      r_wheel_pid.reset();
      l_wheel_pid.reset();
    }
  } else if (jstate == RUNNING) {
    r_wheel_pid.attempt_update(0.5, r_wheel_velocity);
    l_wheel_pid.attempt_update(0.5, l_wheel_velocity);
    motors.set_right_motor(r_wheel_pid.feedback_signal);
    motors.set_left_motor(l_wheel_pid.feedback_signal);
    
    kinematics.attempt_update();
  
    if (line_sensors.is_on_line()) {
      jstate = TRANSITION;
    }
  } else if (jstate == TRANSITION) {
    if (!line_sensors.is_on_line()) {
      jstate = RECORDING;
    }
  } else if (jstate == RECORDING) {
    r_wheel_pid.attempt_update(0.5, r_wheel_velocity);
    l_wheel_pid.attempt_update(0.5, l_wheel_velocity);
    motors.set_right_motor(r_wheel_pid.feedback_signal);
    motors.set_left_motor(l_wheel_pid.feedback_signal);
    
    kinematics.attempt_update();
    results.attempt_update(r_wheel_pid.feedback_signal, r_wheel_velocity);
    if (line_sensors.is_on_line()) {
      jstate = STOPPED;
    }
    
  } else if (jstate == STOPPED) {
    motors.set_motors(0,0);
    if (digitalRead(17) == LOW) {
      results.print_results();
    }
  }


  if (micros() - last_update > 200000) {
    Serial.println("UPDATING");
    Serial.println(count_e0);
    e0_change = count_e0 - last_e0;
    e1_change = count_e1 - last_e1;
    loop_interval = micros() - last_update;
    last_update = micros();
    last_e0 = count_e0;
    last_e1 = count_e1;
    r_wheel_velocity = -1000 * (e0_change / loop_interval);
    l_wheel_velocity = -1000 * (e1_change / loop_interval);
    Serial.println(r_wheel_velocity);
  }
  
}
