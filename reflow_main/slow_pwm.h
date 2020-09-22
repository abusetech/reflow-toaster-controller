#ifndef SLOW_PWM_H
#define SLOW_PWM_H

#include <Arduino.h>

class SlowPWM {
		int duty_cycle = 0;
		int pin = 0;
		int period = 0;
	public:
		SlowPWM(int period, int pin);
		void setDutyCycle(int cycle);
		void update();
                int getHighTime();
                int getPeriod();
};
	
#endif
