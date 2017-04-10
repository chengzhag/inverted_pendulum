#include "encoder_exti.h"

void EncoderExti::eventA()
{
	int state = a_pin->read() << 1 | b_pin->read();
	switch (state)
	{
	case 0:
	case 3:
		position++;
		break;
	case 1:
	case 2:
		position--;
		break;
	}
}

void EncoderExti::eventB()
{
	int state = a_pin->read() << 1 | b_pin->read();
	switch (state)
	{
	case 0:
	case 3:
		position--;
		break;
	case 1:
	case 2:
		position++;
		break;
	}
}

EncoderExti::EncoderExti(Gpio *Apin, Gpio *Bpin) :
	a_pin(Apin), b_pin(Bpin),
	extiA(Apin, EXTI_Trigger_Rising_Falling),
	extiB(Bpin, EXTI_Trigger_Rising_Falling),
	position(0)
{
	extiA.begin();
	extiA.attach(this, &EncoderExti::eventA);
	extiA.interrupt(ENABLE);
	extiB.begin();
	extiB.attach(this, &EncoderExti::eventB);
	extiB.interrupt(ENABLE);
	//Bpin->mode(INPUT);
}

ROTARY_ENCODER_TYPE EncoderExti::getPosition()
{
	return position;
}

void EncoderExti::resetPosition()
{
	position = 0;
}
