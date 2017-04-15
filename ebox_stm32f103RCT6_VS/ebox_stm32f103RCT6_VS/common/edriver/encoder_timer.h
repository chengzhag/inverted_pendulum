#pragma once

#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "ebox.h"

//未重映射定时器ch1和ch2的情况下
//- TIM1 : PA8 PA9
//- TIM2 : PA0 PA1
//- TIM3 : PA6 PA7
//- TIM4 : PB6 PB7
class EncoderTimer
{
	long pos;
	short diff;
	Gpio *pinA;
	Gpio *pinB;
	TIM_TypeDef *timer;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
public:
	EncoderTimer(TIM_TypeDef *TIMx);

	//配置IO和寄存器
	void begin();

	//获取位置
	long getPos();

	//重置位置
	void resetPos();

	//读取timer寄存器并清空
	void refresh();

	//获取速度
	short getDiff();
};