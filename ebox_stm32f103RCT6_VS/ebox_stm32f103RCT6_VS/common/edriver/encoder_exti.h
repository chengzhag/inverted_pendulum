#ifndef  __ENCODER_EXTI
#define  __ENCODER_EXTI

#include "ebox.h"

class EncoderExti
{
	long long position;
	long long positionOld;
	long difference;
	Gpio *a_pin;
	Gpio *b_pin;
	Exti extiA;
	Exti extiB;
	void eventA();
	void eventB();
public:

	//初始化正交编码器，Apin、Bpin分别为编码器的A、B相
	EncoderExti(Gpio *Apin, Gpio *Bpin);

	//开始配置ebox选项
	void begin();

	//获取位置
	long long getPosition();

	//重置位置为0
	void resetPosition();

	//计算与上次位置的差值
	void countDiff();

	//获取计算的差值
	long getDiff();
};




#endif