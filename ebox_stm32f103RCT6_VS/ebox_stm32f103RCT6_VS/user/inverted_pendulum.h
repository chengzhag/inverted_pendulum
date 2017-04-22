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

	//将增量旋转编码器作为绝对编码器使用
	//numPerRound: 每圈pos的增量
	EncoderPendulum(TIM_TypeDef *TIMx,
		unsigned int numPerRound = 2000);

	//获取绝对弧度值，以初始化点为+-pi点，范围-pi~pi
	float getRadian();

	//获取角速度，单位弧度
	float getRadianDiff();
};

class MotorBeam :public EncoderMotor
{
	unsigned int npr;//每圈pos的增量

public:

	//横梁电机，可以获取以弧度为单位的相对角度值
	//numPerRound: 每圈pos的增量
	MotorBeam(TIM_TypeDef *TIMx,
		Gpio *motorPinA, Gpio *motorPinB, Gpio *motorPinPwm,
		unsigned int numPerRound = 1560,
		int controlTarget = Encoder_Motor_Target_Position,
		float refreshInterval = 0.005);

	//获取弧度值，以初始化点为0弧度点，范围-nan~+nan
	float getRadian();

	//获取角速度，单位弧度
	float getRadianDiff();

	//设置目标角度弧度差
	void setRadianDiff(float radian);
};

class InvertedPendulum
{
	float refreshInt;
	float enRadThres;//进行pid反馈的角度范围，单方向，单位弧度。初始pi/4
	bool invertedPIDEnable;
public:
	greg::PID pendulumRadianPID, beamRadianPID,//角度PID
		pendulumPalstancePID, beamPalstancePID;//角速度PID
	EncoderPendulum encoder;
	MotorBeam motor;

	InvertedPendulum(TIM_TypeDef *TIMpendulum, TIM_TypeDef *TIMmotor,
		Gpio *motorPinA, Gpio *motorPinB, Gpio *motorPinPwm,
		unsigned int nprPendulum = 2000, unsigned int nprMotor = 1560, float refreshInterval = 0.005);

	//初始化编码器、电机、PID控制，设置初始化PID值
	void begin();

	//对编码器、电机PID、倒立PID进行刷新
	void refresh();

	//设置倒立PID使能
	void setInvertedPIDEnable(bool b);

	//设置进行pid反馈的角度范围
	void setEnRadThres(float t);
};

#endif