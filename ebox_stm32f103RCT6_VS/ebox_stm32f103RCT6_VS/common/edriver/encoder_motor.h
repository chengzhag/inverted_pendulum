#ifndef  __ENCODER_MOTOR
#define  __ENCODER_MOTOR

#include "ebox.h"
#include "PID.hpp"
#include "encoder_exti.h"
#include "tb6612fng.h"

enum encoderMotorControlTarget
{
	Encoder_Motor_Target_Position,
	Encoder_Motor_Target_Speed
};

class EncoderMotor
{
	EncoderExti encoder;
	TB6612FNG driver;
	int mode;
public:
	greg::PID pid;
	EncoderMotor(Gpio *enA, Gpio *enB, 
		Gpio *moA, Gpio *moB, Gpio *moPwm,
		int controlTarget = Encoder_Motor_Target_Position) :
		encoder(enA, enB),
		driver(moA, moB, moPwm),
		mode(controlTarget)
	{
		switch (controlTarget)
		{
		case Encoder_Motor_Target_Position:

			break;
		case Encoder_Motor_Target_Speed:

			break;
		default:
			break;
		}
	}
};

#endif
