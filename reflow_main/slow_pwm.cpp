#ifndef SLOW_PWM_CPP
#define SLOW_PWM_CPP

#include "slow_pwm.h"

SlowPWM::SlowPWM(int period, int pin){
	pinMode(pin, OUTPUT);
	this->pin = pin;
	this->period = period;
}

void SlowPWM::setDutyCycle(int cycle){
	duty_cycle = cycle;
}

int SlowPWM::getHighTime(){
	return duty_cycle;
}

int SlowPWM::getPeriod(){
	return period;
}

void SlowPWM::update(){
	int period_time = millis() % period;
	if (period_time < duty_cycle){
		digitalWrite(pin, HIGH);	
	}else{
		digitalWrite(pin, LOW);
	}
}

#endif
