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

	EncoderMotor(TIM_TypeDef *TIMx,
		Gpio *motorPinA, Gpio *motorPinB, Gpio *motorPinPwm,
		int controlTarget = Encoder_Motor_Target_Position,
		float refreshInterval = 0.01);
	void begin();
	void begin(const float &Kp, const float &Ki, const float &Kd);
	void refresh();
	long getPos();
	long getSpd();
	float getPercent();
	void setPos(long pos);
	void setSpd(long spd);
};

#endif
