#include "inverted_pendulum.h"

EncoderPendulum::EncoderPendulum(TIM_TypeDef *TIMx, unsigned int numPerRound /*= 2000*/) :
	EncoderTimer(TIMx),
	npr(numPerRound)
{

}

float EncoderPendulum::getRadian()
{
	long posTemp = getPos();
	if (posTemp >= 0)
	{
		return getPos() % npr / (float)npr * 2 * M_PI - M_PI;
	}
	else
	{
		return (npr - (-getPos()) % npr) / (float)npr * 2 * M_PI - M_PI;
	}
}

float EncoderPendulum::getRadianDiff()
{
	return getDiff() / (float)npr * 2 * M_PI;
}

MotorBeam::MotorBeam(TIM_TypeDef *TIMx, Gpio *motorPinA, Gpio *motorPinB, Gpio *motorPinPwm, unsigned int numPerRound /*= 1560*/, int controlTarget /*= Encoder_Motor_Target_Position*/, float refreshInterval /*= 0.005*/) :
	EncoderMotor(TIMx, motorPinA, motorPinB, motorPinPwm,
		controlTarget, refreshInterval),
	npr(numPerRound)
{

}

double MotorBeam::getRadian()
{
	return getPos() / (double)npr * 2 * M_PI;
}

float MotorBeam::getRadianDiff()
{
	return getPosDiff() / (float)npr * 2 * M_PI;
}

void MotorBeam::setRadianDiff(float radian)
{
	setPosDiff(radian / 2 / M_PI*npr);
}

InvertedPendulum::InvertedPendulum(TIM_TypeDef *TIMpendulum, TIM_TypeDef *TIMmotor, Gpio *motorPinA, Gpio *motorPinB, Gpio *motorPinPwm, unsigned int nprPendulum /*= 2000*/, unsigned int nprMotor /*= 1560*/, float refreshInterval /*= 0.005*/) :
	encoder(TIMpendulum, nprPendulum),
	motor(TIMmotor, motorPinA, motorPinB, motorPinPwm,
		nprMotor, Encoder_Motor_Target_Position, refreshInterval),
	refreshInt(refreshInterval),
	enRadThres(M_PI / 4),
	invertedPIDEnable(false)
{

}

void InvertedPendulum::begin()
{
	encoder.begin();
	motor.begin(1.8, 1.75, 0.05);
	setInvertedPIDEnable(true);

	//TODO: 修正角度限制、pid调参，将反馈值单位改成弧度
	//初始化摆杆角度PID
	pendulumRadianPID.setRefreshInterval(refreshInt);
	pendulumRadianPID.setWeights(1.5, 2, 0.006);
	pendulumRadianPID.setOutputLowerLimit(-2);
	pendulumRadianPID.setOutputUpperLimit(2);
	pendulumRadianPID.setDesiredPoint(0);

	//初始化衡量位置PID
	beamRadianPID.setRefreshInterval(refreshInt);
	//beamPID.setWeights(0.018, 0, 0.006);
	beamRadianPID.setWeights(0, 0, 0);
	beamRadianPID.setOutputLowerLimit(-0.5);
	beamRadianPID.setOutputUpperLimit(0.5);
	beamRadianPID.setDesiredPoint(0);
}

void InvertedPendulum::refresh()
{
	float desiredRadianPendulum = 0;

	//刷新编码器和电机位置PID
	encoder.refresh();
	motor.refresh();

	float radianPendulum = -encoder.getRadian();
	if (radianPendulum < enRadThres && radianPendulum>-enRadThres && invertedPIDEnable)
	{
		//横梁位置
		desiredRadianPendulum = beamRadianPID.refresh(-motor.getRadian());
		pendulumRadianPID.setDesiredPoint(desiredRadianPendulum);
		//摆杆角度PID
		motor.setRadianDiff(pendulumRadianPID.refresh(radianPendulum));
	}
}

void InvertedPendulum::setInvertedPIDEnable(bool b)
{
	invertedPIDEnable = b;
}

void InvertedPendulum::setEnRadThres(float t)
{
	if (t < M_PI / 2 && t>0)
	{
		enRadThres = t;
	}
}


