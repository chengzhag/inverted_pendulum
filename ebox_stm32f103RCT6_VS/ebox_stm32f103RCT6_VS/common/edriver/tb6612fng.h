#ifndef  __TB6612FNG
#define  __TB6612FNG

#include "ebox.h"
#include <math.h>

class TB6612FNG
{
	Gpio *aPin;
	Gpio *bPin;
	Gpio *pwmPin;
	Pwm pwm;
	uint32_t frq;
	float percent;
public:
	TB6612FNG(Gpio *pinA, Gpio *pinB, 
		Gpio *pinPwm, uint32_t frequency = 15000);
	void begin();
	void setPercent(float p);
	float getPercent();
};




#endif