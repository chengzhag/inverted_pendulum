/**
  ******************************************************************************
  * @file   : *.cpp
  * @author : shentq
  * @version: V1.2
  * @date   : 2016/08/14

  * @brief   ebox application example .
  *
  * Copyright 2016 shentq. All Rights Reserved.         
  ******************************************************************************
 */

#include "ebox.h"

#define ROTARY_ENCODER_TYPE long long
class RotaryEncoder
{
	ROTARY_ENCODER_TYPE position;
	Gpio *a_pin;
	Gpio *b_pin;
	Exti extiA;
	void eventA() {
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
public:
	RotaryEncoder(Gpio *Apin, Gpio *Bpin) :
		a_pin(Apin), b_pin(Bpin),
		extiA(Apin, EXTI_Trigger_Rising_Falling),
		position(0)
	{
		extiA.begin();
		extiA.attach(this,&RotaryEncoder::eventA);
		extiA.interrupt(ENABLE);
		Bpin->mode(INPUT);
	}
	ROTARY_ENCODER_TYPE getPosition() 
	{
		return position;
	}
	void resetPosition() 
	{
		position = 0;
	}

};

RotaryEncoder encoder1(&PA8, &PA7);

void setup()
{
    ebox_init();
    uart1.begin(115200);
  
}

int main(void)
{
    setup();
	long long oldPos = 0, nowPos = 0;

    while(1)
    {
		nowPos = encoder1.getPosition();
        uart1.printf("%ld\n", nowPos- oldPos);
		oldPos = nowPos;
        delay_ms(100);

    }

}


