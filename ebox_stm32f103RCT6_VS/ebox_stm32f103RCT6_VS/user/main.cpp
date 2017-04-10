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
#include "encoder.h"

#define ROTARY_ENCODER_TYPE long long
class RotaryEncoder
{
	ROTARY_ENCODER_TYPE position;
	Encoder encoder;
	Exti extiA;
	void eventA() {
		switch (encoder.read_encoder())
		{
		case 0:
			break;
		case 1:
			position--;
			break;
		case 2:
			position++;
			break;
		}
	}
public:
	RotaryEncoder(Gpio *Apin, Gpio *Bpin) :
		encoder(Apin, Bpin) ,
		extiA(Apin, EXTI_Trigger_Falling),
		position(0)
	{
		extiA.begin();
		extiA.attach(this,&RotaryEncoder::eventA);
		extiA.interrupt(ENABLE);
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
    while(1)
    {
		
        uart1.printf("%ld\n", encoder1.getPosition());
        delay_ms(50);

    }

}


