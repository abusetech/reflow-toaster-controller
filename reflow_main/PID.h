#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID {

  unsigned long last_time;
  float i_term, last_input, max_output;
  float kp, ki, kd;
  boolean automatic = false;
  
  public:
    PID(float kp, float ki, float kd, float max_output);
    float compute(float input, float setpoint);
    void setTunings(float kp, float ki, float kd);
    void init(float input, float output);
};

#endif
