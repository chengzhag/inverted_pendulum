#ifndef INVERTED_PENDULUM
#define INVERTED_PENDULUM

#include "encoder_timer.h"
#include "encoder_motor.h"
#include "PID.hpp"

#define PID_REFRESH_INTERVAL 0.005
#define M_PI		3.14159265358979323846


class EncoderPendulum :public EncoderTimer
{
	unsigned int npr;//每圈pos的增量

public:

	//numPerRound: 每圈pos的增量
	EncoderPendulum(TIM_TypeDef *TIMx,
		unsigned int numPerRound = 2000) :
		EncoderTimer(TIMx),
		npr(numPerRound)
	{}

	////以初始化点为+-180度点的绝对角度值，范围-180~180，单位度
	//float getDegree()
	//{
	//	return getPos() % npr / (float)npr * 360 - 180;
	//}

	//以初始化点为+-pi点的绝对弧度值，范围-pi~pi，单位弧度
	float getRadian()
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
};

class MotorBeam :public EncoderMotor
{
	unsigned int npr;//每圈pos的增量

public:

	//numPerRound: 每圈pos的增量
	MotorBeam(TIM_TypeDef *TIMx,
		Gpio *motorPinA, Gpio *motorPinB, Gpio *motorPinPwm,
		unsigned int numPerRound = 1560,
		int controlTarget = Encoder_Motor_Target_Position,
		float refreshInterval = 0.005) :
		EncoderMotor(TIMx, motorPinA, motorPinB, motorPinPwm,
			controlTarget, refreshInterval),
		npr(numPerRound)
	{}

	////以初始化点为0度点的绝对角度值，范围-nan~+nan，单位度
	//double getDegree()
	//{
	//	return getPos() / (double)npr * 360;
	//}

	//以初始化点为0弧度点的绝对角度值，范围-nan~+nan，单位弧度
	double getRadian()
	{
		return getPos() / (double)npr * 2 * M_PI;
	}

	//设置目标角度弧度差
	void setRadianDiff(float radian)
	{
		setPosDiff(radian / 2 / M_PI*npr);
	}
};

class InvertedPendulum
{

	greg::PID pendulumPID, beamPID;
	float refreshInt;
	const float enableRadian;//进行pid反馈的角度范围，单方向，单位弧度
	bool enableInvertedPID;
public:
	EncoderPendulum encoder;
	MotorBeam motor;

	InvertedPendulum(TIM_TypeDef *TIMpendulum, TIM_TypeDef *TIMmotor,
		Gpio *motorPinA, Gpio *motorPinB, Gpio *motorPinPwm,
		unsigned int nprPendulum = 2000, unsigned int nprMotor = 1560, float refreshInterval = 0.005) :
		encoder(TIMpendulum, nprPendulum),
		motor(TIMmotor, motorPinA, motorPinB, motorPinPwm,
			nprMotor, Encoder_Motor_Target_Position, refreshInterval),
		refreshInt(refreshInterval),
		enableRadian(M_PI / 4),
		enableInvertedPID(false)
	{}

	void begin()
	{
		encoder.begin();
		motor.begin(1.8, 1.75, 0.05);
		setEnableInvertedPID(true);

		//TODO: 修正角度限制、pid调参，将反馈值单位改成弧度
		//初始化摆杆角度PID
		pendulumPID.setRefreshInterval(refreshInt);
		pendulumPID.setWeights(1.5, 2, 0.006);
		pendulumPID.setOutputLowerLimit(-2);
		pendulumPID.setOutputUpperLimit(2);
		pendulumPID.setDesiredPoint(0);

		//初始化衡量位置PID
		beamPID.setRefreshInterval(refreshInt);
		//beamPID.setWeights(0.018, 0, 0.006);
		beamPID.setWeights(0, 0, 0);
		beamPID.setOutputLowerLimit(-0.5);
		beamPID.setOutputUpperLimit(0.5);
		beamPID.setDesiredPoint(0);
	}

	void refresh()
	{
		float desiredRadianPendulum = 0;

		//刷新编码器和电机位置PID
		encoder.refresh();
		motor.refresh();

		float radianPendulum = -encoder.getRadian();
		if (radianPendulum < enableRadian && radianPendulum>-enableRadian && enableInvertedPID)
		{
			//横梁位置
			desiredRadianPendulum = beamPID.refresh(-motor.getRadian());
			pendulumPID.setDesiredPoint(desiredRadianPendulum);
			//摆杆角度PID
			motor.setRadianDiff(pendulumPID.refresh(radianPendulum));
		}
	}

	void setEnableInvertedPID(bool b)
	{
		enableInvertedPID = b;
	}
};

#endif