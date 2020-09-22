#ifndef PID_CPP
#define PID_CPP

#include "PID.h"
#include <Arduino.h>

PID::PID(float kp, float ki, float kd, float max_output){
  
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
  this->max_output = max_output;
}

float PID::compute(float in, float set){

  float error = set - in;
  
  unsigned long now = millis();
  float time_change = (float)(now - last_time);
  
  i_term += error * time_change * ki;
  if(i_term > max_output){i_term = max_output;}
  else if (i_term < 0){i_term = 0;}
  //derror/dt = - dinput/dt
  float d_error = -(in - last_input)/time_change;
  float output = kp * error + i_term + kd * d_error;
  
  last_time = now;
  last_input = in;
  
  if (output < 0){output = 0;}
  else if (output > max_output){output = max_output;}
  
  return output;

}

void PID::setTunings(float kP, float kI, float kD){
  
  kp = kP;
  ki = kI;
  kd = kD;
  
}

void PID::init(float input, float output){
  //input is our current sensor input
  last_input = input;
  i_term = output;
  if(i_term > max_output){i_term = max_output;}
  else if (i_term < 0){i_term = 0;} 
}

#endif
