#include "inverted_pendulum.h"

EncoderPendulum::EncoderPendulum(TIM_TypeDef *TIMx, unsigned int numPerRound /*= 2000*/) :
	EncoderTimer(TIMx),
	npr(numPerRound)
{

}

void EncoderPendulum::refresh()
{
	oldDiff = getDiff();
	EncoderTimer::refresh();
	ddiff = getDiff() - oldDiff;
}

float EncoderPendulum::getRadian()
{
	long posTemp = getPos();
	if (posTemp >= 0)
	{
		return getPos() % npr / (float)npr * 2 * PI - PI;
	}
	else
	{
		return (npr - (-getPos()) % npr) / (float)npr * 2 * PI - PI;
	}
}

float EncoderPendulum::getRadianDiff()
{
	return getDiff() / (float)npr * 2 * PI;
}

float EncoderPendulum::getRadianDDiff()
{
	return ddiff / (float)npr * 2 * PI;
}

MotorBeam::MotorBeam(TIM_TypeDef *TIMx, Gpio *motorPinA, Gpio *motorPinB,
	Gpio *motorPinPwm, unsigned int numPerRound /*= 1560*/,
	Encoder_Motor_Target_Typedef controlTarget /*= Encoder_Motor_Target_Position*/, float refreshInterval /*= 0.005*/) :
	EncoderMotor(TIMx, motorPinA, motorPinB, motorPinPwm,
		controlTarget, refreshInterval),
	npr(numPerRound)
{

}

float MotorBeam::getRadian()
{
	return getPos() / (double)npr * 2 * PI;
}

float MotorBeam::getRadianDiff()
{
	return getPosDiff() / (float)npr * 2 * PI;
}


InvertedPendulum::InvertedPendulum(TIM_TypeDef *TIMpendulum, 
	TIM_TypeDef *TIMmotor, Gpio *motorPinA, Gpio *motorPinB, Gpio *motorPinPwm,
	unsigned int nprPendulum /*= 2000*/, unsigned int nprMotor /*= 1560*/, float refreshInterval /*= 0.005*/) :
	encoder(TIMpendulum, nprPendulum),
	motor(TIMmotor, motorPinA, motorPinB, motorPinPwm,
		nprMotor, Encoder_Motor_PID_Disabled, refreshInterval),
	refreshInt(refreshInterval),
	enRadThres(PI / 3),
	mode(Inverted_Pendulum_Mode_Disabled),
	targetBeamPalstance(0),
	targetBeamRadian(0)
{

}

void InvertedPendulum::begin()
{
	encoder.begin();
	motor.begin();

	//初始化摆杆角度PID
	pendulumRadianPID.setRefreshInterval(refreshInt);
	pendulumRadianPID.setWeights(700, 1400, 0);
	pendulumRadianPID.setOutputLowerLimit(-INF_FLOAT);
	pendulumRadianPID.setOutputUpperLimit(INF_FLOAT);
	pendulumRadianPID.setDesiredPoint(0);

	//初始化摆杆角速度PID
	pendulumPalstancePID.setRefreshInterval(refreshInt);
	pendulumPalstancePID.setWeights(10, 0, 0);
	pendulumPalstancePID.setOutputLowerLimit(-INF_FLOAT);
	pendulumPalstancePID.setOutputUpperLimit(INF_FLOAT);
	pendulumPalstancePID.setDesiredPoint(0);

	//初始化横梁角度PID
	beamRadianPID.setRefreshInterval(refreshInt);
	//beamRadianPID.setWeights(200, 100, 0);//反应快PID
	beamRadianPID.setWeights(60, 40, 0);//抗击打PID
	//beamRadianPID.setWeights(0, 0, 0);
	beamRadianPID.setOutputLowerLimit(-INF_FLOAT);
	beamRadianPID.setOutputUpperLimit(INF_FLOAT);
	beamRadianPID.setDesiredPoint(0);

	//初始化横梁角速度PID
	beamPalstancePID.setRefreshInterval(refreshInt);
	//beamPalstancePID.setWeights(6, 10, 0);//反应快PID
	beamPalstancePID.setWeights(1, 0.1, 0);//抗击打PID
	//beamPalstancePID.setWeights(0, 0, 0);
	beamPalstancePID.setOutputLowerLimit(-INF_FLOAT);
	beamPalstancePID.setOutputUpperLimit(INF_FLOAT);
	beamPalstancePID.setDesiredPoint(0);
}

void InvertedPendulum::refresh()
{
	//刷新编码器和电机位置PID
	encoder.refresh();
	motor.refresh();

	float pendulumRadian = getPendulumRadian();

	if (mode == Inverted_Pendulum_Mode_Disabled)
	{
		resetPID();
		setTargetBeamRadian(getBeamRadian());
		motor.setPercent(0);
	}
	else if (mode==Inverted_Pendulum_Mode_Swing_Begin)
	{
		motor.setPercent(50);
		if (pendulumRadian < PI/1.5 && pendulumRadian>-PI / 1.5)
		{
			setMode(Inverted_Pendulum_Mode_Swing);
		}
	}
	else if (mode == Inverted_Pendulum_Mode_Swing)
	{
		refreshSwing();
		if ((pendulumRadian > PI*0.8 || pendulumRadian < -PI*0.8)
			&& getBeamPalstance() == 0)
		{
			setMode(Inverted_Pendulum_Mode_Swing_Begin);
		}
	}
	else if (mode == Inverted_Pendulum_Mode_Invert)
	{
		if (pendulumRadian < enRadThres && pendulumRadian>-enRadThres)
		{
			beamPalstancePID.setDesiredPoint(0);
			refreshPID();
		}
		else
		{
			resetPID();
			setTargetBeamRadian(getBeamRadian());
			motor.setPercent(0);
		}
	}
	else if (mode == Inverted_Pendulum_Mode_Invert_Swing)
	{
		if (pendulumRadian < enRadThres && pendulumRadian>-enRadThres)
		{
			beamPalstancePID.setDesiredPoint(0);
			refreshPID();
		}
		else
		{
			setMode(Inverted_Pendulum_Mode_Swing_Invert);
		}
	}
	else if (mode==Inverted_Pendulum_Mode_Swing_Invert_Begin)
	{
		motor.setPercent(50);
		if (pendulumRadian < PI / 1.5 && pendulumRadian>-PI / 1.5)
		{
			setMode(Inverted_Pendulum_Mode_Swing_Invert);
		}
	}
	else if (mode == Inverted_Pendulum_Mode_Swing_Invert)
	{
		refreshSwing();
		if (pendulumRadian < PI / 12 && pendulumRadian>-PI / 12)
		{
			setMode(Inverted_Pendulum_Mode_Invert_Swing);
			resetPID();
			setTargetBeamRadian(getBeamRadian());
		}
		if ((pendulumRadian > PI*0.8 || pendulumRadian < -PI*0.8)
			&& abs(getBeamPalstance()) < 0.1)
		{
			setMode(Inverted_Pendulum_Mode_Swing_Invert_Begin);
		}
	}
	else if (mode == Inverted_Pendulum_Mode_Round)
	{
		if (pendulumRadian < enRadThres && pendulumRadian>-enRadThres)
		{
			//设置横梁目标速度
			//对横梁实际角度与目标角度进行判断
			if (abs(getBeamRadian() - getTargetBeamRadian()) < PI)
			{
				setTargetBeamRadian(getTargetBeamRadian() + targetBeamPalstance*refreshInt);
			}
			else
			{
				if (getBeamRadian() - getTargetBeamRadian() > 0)
				{
					setTargetBeamRadian(getBeamRadian() - PI);
				}
				else
				{
					setTargetBeamRadian(getBeamRadian() + PI);
				}
			}
			beamRadianPID.setDesiredPoint(getTargetBeamRadian());
			beamPalstancePID.setDesiredPoint(targetBeamPalstance);

			refreshPID();
		}
		else
		{
			resetPID();
			setTargetBeamRadian(getBeamRadian());
			motor.setPercent(0);
		}
	}
}


void InvertedPendulum::refreshPID()
{
	float motorPercent = 0;

	//获取横梁、摆杆角度、角速度
	float pendulumRadian = getPendulumRadian();
	float pendulumPalstance = getPendulumPalstance();
	float beamRadian = getBeamRadian();
	float beamPalstance = getBeamPalstance();

	//计算四个PID的输出，设置+-以改变控制方向
	motorPercent -= pendulumRadianPID.refresh(pendulumRadian);
	motorPercent -= pendulumPalstancePID.refresh(pendulumPalstance);
	motorPercent -= beamRadianPID.refresh(beamRadian);
	motorPercent -= beamPalstancePID.refresh(beamPalstance);

	//输出横梁角度增量
	motor.setPercent(motorPercent);
}

void InvertedPendulum::refreshSwing()
{
	float motorPercent = 0;
	float pendulumAcceleration = getPendulumAcceleration();
	//正反馈控制起摆
	motorPercent -= 0.5*pendulumAcceleration;
	//输出横梁角度增量
	motor.setPercent(motorPercent);
}

void InvertedPendulum::setMode(Inverted_Pendulum_Mode_Typedef m)
{
	mode = m;
}

void InvertedPendulum::setEnRadThres(float t)
{
	if (t < PI / 2 && t>0)
	{
		enRadThres = t;
	}
}

float InvertedPendulum::getPendulumRadian()
{
	return encoder.getRadian();
}

float InvertedPendulum::getPendulumPalstance()
{
	return encoder.getRadianDiff() / refreshInt;
}

float InvertedPendulum::getPendulumAcceleration()
{
	return encoder.getRadianDDiff() / refreshInt / refreshInt;
}

float InvertedPendulum::getBeamRadian()
{
	return motor.getRadian();
}

float InvertedPendulum::getBeamPalstance()
{
	return motor.getRadianDiff() / refreshInt;
}

void InvertedPendulum::resetPID()
{
	pendulumRadianPID.reset();
	beamRadianPID.reset();
	pendulumPalstancePID.reset();
	beamPalstancePID.reset();

	pendulumRadianPID.setDesiredPoint(0);
	setTargetBeamRadian(0);
	pendulumPalstancePID.setDesiredPoint(0);
	beamPalstancePID.setDesiredPoint(0);
}

void InvertedPendulum::setTargetBeamPalstance(float desiredBeamPalstance)
{
	targetBeamPalstance = desiredBeamPalstance;
	beamPalstancePID.setDesiredPoint(desiredBeamPalstance);
}

void InvertedPendulum::setTargetBeamRadian(float desiredBeamRadian)
{
	targetBeamRadian = desiredBeamRadian;
	beamRadianPID.setDesiredPoint(desiredBeamRadian);
}

float InvertedPendulum::getTargetBeamRadian()
{
	return targetBeamRadian;
}

