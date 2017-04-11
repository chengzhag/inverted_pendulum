#include "tb6612fng.h"

TB6612FNG::TB6612FNG(Gpio *pinA, Gpio *pinB,
	Gpio *pinPwm, uint32_t frequency /*= 15000*/) :
	aPin(pinA),
	bPin(pinB),
	pwmPin(pinPwm),
	pwm(pinPwm),
	frq(frequency),
	percent(0)
{

}

void TB6612FNG::begin()
{
	aPin->mode(OUTPUT_PP);
	bPin->mode(OUTPUT_PP);
	pwm.begin(frq, 0);
	pwm.set_oc_polarity(1);
}

void TB6612FNG::setPercent(float p)
{
	if (p>100)
	{
		percent = 100;
	}
	else if (p < -100)
	{
		percent = -100;
	}
	else
	{
		percent = p;
	}
	
	uint16_t duty = abs(percent * 10);
	if (p > 0)
	{
		aPin->set();
		bPin->reset();
	}
	else if (p < 0)
	{
		bPin->set();
		aPin->reset();
	}
	else
	{
		bPin->reset();
		aPin->reset();
	}
	pwm.set_duty(duty);
}

float TB6612FNG::getPercent()
{
	return percent;
}
