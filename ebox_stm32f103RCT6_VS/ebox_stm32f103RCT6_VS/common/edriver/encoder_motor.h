#ifndef  __ENCODER_MOTOR
#define  __ENCODER_MOTOR

#include "ebox.h"
#include "PID.hpp"
#include "encoder_timer.h"
#include "tb6612fng.h"

typedef enum
{
	Encoder_Motor_Target_Position,
	Encoder_Motor_Target_Speed
}Encoder_Motor_Target_Typedef;

class EncoderMotor
{
	EncoderTimer encoder;
	TB6612FNG driver;
	int mode;
	float percent;
public:
	greg::PID pid;

	//未重映射的情况下
	//- TIM1 : PA8 PA9
	//- TIM2 : PA0 PA1
	//- TIM3 : PA6 PA7
	//- TIM4 : PB6 PB7
	EncoderMotor(TIM_TypeDef *TIMx,
		Gpio *motorPinA, Gpio *motorPinB, Gpio *motorPinPwm,
		int controlTarget = Encoder_Motor_Target_Position,
		float refreshInterval = 0.01);

	void begin();
	void begin(const float &Kp, const float &Ki, const float &Kd);
	void refresh();
	long getPos();
	short getSpd();
	float getPercent();
	void setPos(long pos);
	void setPosDiff(int pos);
	void setSpd(long spd);
};

#endif
