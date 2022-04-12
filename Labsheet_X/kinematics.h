// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#define CPR 358.3
#define WHEEL_RADIUS 16
#define WHEEL_SEPERATION 80

#include "encoders.h"


// Class to track robot position.
class Kinematics_c {
  public:
    double count_r = 0;
    double count_l = 0;
    double x_i = 0;
    double y_i = 0;
    double r_i = 0;
    double x_r = 0;
    double r_r = 0;
    double time_interval = 50;
    double last_update = 0;
  
    // Constructor, must exist.
    Kinematics_c() {
      
    } 

    double mod_degrees(double angle) {
      if (angle >= 360) {
        return angle - 360;
      } else if (angle < 0) {
        return angle + 360;
      } else {
        return angle;
      }
    }

    void attempt_update() {
      if (time_interval < (millis() - last_update)) {
        update(count_e0 - count_l, count_e1 - count_r);
        count_l = count_e0;
        count_r = count_e1;
        last_update = millis();
//        Serial.println("Values");
//        Serial.println(count_e0 - count_l);
//        Serial.println(x_i);
//        Serial.println(y_i);
//        Serial.println(r_i);
      }
    }

    // Use this function to update
    // your kinematics
    void update(double tick_l, double tick_r) {
      double r_dist = tick_r * WHEEL_RADIUS;
      double l_dist = tick_l * WHEEL_RADIUS;
      x_r = r_dist/(double)2 + l_dist/(double)2;
      r_r = r_dist/(double)WHEEL_SEPERATION - l_dist/(double)WHEEL_SEPERATION;
      x_i = x_i + x_r * cos(radians(r_i));
      y_i = y_i + x_r * sin(radians(r_i));
      r_i = mod_degrees(r_i + r_r);
    }

};



#endif
