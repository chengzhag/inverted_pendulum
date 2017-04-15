#include "encoder_motor.h"

EncoderMotor::EncoderMotor(TIM_TypeDef *TIMx, Gpio *motorPinA, Gpio *motorPinB, Gpio *motorPinPwm, int controlTarget /*= Encoder_Motor_Target_Position*/, float refreshInterval /*= 0.01*/) :
	encoder(TIMx),
	driver(motorPinA, motorPinB, motorPinPwm),
	mode(controlTarget),
	percent(0)
{
	switch (controlTarget)
	{
	case Encoder_Motor_Target_Position:
		pid.setRefreshInterval(refreshInterval);
		pid.setWeights(1.5, 0.1, 0.02);
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

void EncoderMotor::begin(const float &Kp, const float &Ki, const float &Kd)
{
	pid.setWeights(Kp, Ki, Kd);
	begin();
}

void EncoderMotor::begin()
{
	driver.begin();
	encoder.begin();
}

void EncoderMotor::refresh()
{
	encoder.refresh();
	percent = pid.refresh(encoder.getPos());
	driver.setPercent(percent);
}

long EncoderMotor::getPos()
{
	return encoder.getPos();
}

short EncoderMotor::getSpd()
{
	return encoder.getDiff();
}

float EncoderMotor::getPercent()
{
	return percent;
}

void EncoderMotor::setPos(long pos)
{
	if (mode == Encoder_Motor_Target_Position)
	{
		pid.setDesiredPoint(pos);
	}
}

void EncoderMotor::setSpd(long spd)
{
	if (mode == Encoder_Motor_Target_Speed)
	{
		pid.setDesiredPoint(spd);
	}
}
