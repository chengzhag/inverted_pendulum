#ifndef  ENCODER_EXTI
#define  ENCODER_EXTI

#include "ebox.h"

#define ROTARY_ENCODER_TYPE long long
class EncoderExti
{
	ROTARY_ENCODER_TYPE position;
	Gpio *a_pin;
	Gpio *b_pin;
	Exti extiA;
	Exti extiB;
	void eventA();
	void eventB();
public:
	EncoderExti(Gpio *Apin, Gpio *Bpin);
	ROTARY_ENCODER_TYPE getPosition();
	void resetPosition();
};




#endif