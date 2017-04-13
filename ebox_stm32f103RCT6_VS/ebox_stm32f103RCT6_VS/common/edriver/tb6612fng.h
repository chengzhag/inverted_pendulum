#ifndef  __TB6612FNG
#define  __TB6612FNG

#include "ebox.h"
#include <math.h>

class TB6612FNG
{
	Gpio *pinA;
	Gpio *pinB;
	Gpio *pinP;
	Pwm pwm;
	uint32_t frq;
	float pct;
public:
	TB6612FNG(Gpio *motorPinA, Gpio *motorPinB,
		Gpio *motorPinPwm, uint32_t pwmFrequency = 15000);
	void begin();
	void setPercent(float p);
	float getPercent();
};




#endif