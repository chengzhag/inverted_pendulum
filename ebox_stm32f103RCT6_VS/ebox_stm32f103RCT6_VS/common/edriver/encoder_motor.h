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
	float percent;
public:
	greg::PID pid;

	EncoderMotor(Gpio *encoderPinA, Gpio *encoderPinB, 
		Gpio *motorPinA, Gpio *motorPinB, Gpio *motorPinPwm,
		int controlTarget = Encoder_Motor_Target_Position,
		float refreshInterval = 0.01) :
		encoder(encoderPinA, encoderPinB),
		driver(motorPinA, motorPinB, motorPinPwm),
		mode(controlTarget),
		percent(0)
	{
		switch (controlTarget)
		{
		case Encoder_Motor_Target_Position:
			pid.setRefreshInterval(refreshInterval);
			pid.setWeights(0.8, 0, 0);
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
	void begin(const float &Kp, const float &Ki, const float &Kd)
	{
		pid.setWeights(Kp, Ki, Kd);
		begin();
	}
	void refresh()
	{
		encoder.countDiff();
		percent = pid.refresh(encoder.getPos());
		driver.setPercent(percent);
	}
	long getPos()
	{
		return encoder.getPos();
	}
	long getSpd()
	{
		return encoder.getDiff();
	}
	float getPercent()
	{
		return percent;
	}
	void setPos(long pos)
	{
		if (mode==Encoder_Motor_Target_Position)
		{
			pid.setDesiredPoint(pos);
		}
	}
	void setSpd(long spd)
	{
		if (mode == Encoder_Motor_Target_Speed)
		{
			pid.setDesiredPoint(spd);
		}
	}
};

#endif
