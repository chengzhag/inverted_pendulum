#ifndef  __ENCODER_MOTOR
#define  __ENCODER_MOTOR

#include "ebox.h"
#include "PID.hpp"
#include "encoder_exti.h"
#include "tb6612fng.h"

typedef enum
{
	Encoder_Motor_Target_Position,
	Encoder_Motor_Target_Speed
}Encoder_Motor_Target_Typedef;

class EncoderMotor
{
	EncoderExti encoder;
	TB6612FNG driver;
	int mode;
	float outputPercent;
public:
	greg::PID pid;

	EncoderMotor(Gpio *enA, Gpio *enB, 
		Gpio *moA, Gpio *moB, Gpio *moPwm,
		int controlTarget = Encoder_Motor_Target_Position,
		float refreshInterval = 0.01) :
		encoder(enA, enB),
		driver(moA, moB, moPwm),
		mode(controlTarget),
		outputPercent(0)
	{
		switch (controlTarget)
		{
		case Encoder_Motor_Target_Position:
			pid.setRefreshInterval(refreshInterval);
			pid.setWeights(0.5, 0, 0);
			pid.setOutputLowerLimit(-100);
			pid.setOutputUpperLimit(100);
			pid.setDesiredPoint(0);
			break;
		case Encoder_Motor_Target_Speed:

			break;
		default:
			break;
		}
	}
	void begin()
	{
		driver.begin();
		encoder.begin();
	}
	void refresh()
	{
		encoder.countDiff();
		outputPercent = pid.refresh(encoder.getPosition());
		driver.setPercent(outputPercent);
	}
	long long getPosition()
	{
		return encoder.getPosition();
	}
	long getSpeed()
	{
		return encoder.getDiff();
	}
	float getOutputPercent()
	{
		return outputPercent;
	}
	void setPosition(long long pos)
	{
		if (mode==Encoder_Motor_Target_Position)
		{
			pid.setDesiredPoint(pos);
		}
	}
	void setSpeed(long spd)
	{
		if (mode == Encoder_Motor_Target_Speed)
		{
			pid.setDesiredPoint(spd);
		}
	}
};

#endif
