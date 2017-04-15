#pragma once

#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "ebox.h"

class EncoderTimer
{
	short pos;
	short posOld;
	short diff;
	Gpio *pinA;
	Gpio *pinB;
	TIM_TypeDef *timer;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
public:
	//未重映射的情况下
	//- TIM1 : PA8 PA9
	//- TIM2 : PA0 PA1
	//- TIM3 : PA6 PA7
	//- TIM4 : PB6 PB7
	EncoderTimer(TIM_TypeDef *TIMx):
		pos(0), posOld(0)
	{
		timer = TIMx;

		if (timer==TIM1)
		{
			pinA = &PA8;
			pinB = &PA9;
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
		}
		if (timer == TIM2)
		{
			pinA = &PA0;
			pinB = &PA1;
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
		}
		if (timer == TIM3)
		{
			pinA = &PA6;
			pinB = &PA7;
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
		}
		if (timer == TIM4)
		{
			pinA = &PB6;
			pinB = &PB7;
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		}

		TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
		TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // 预分频器 
		TIM_TimeBaseStructure.TIM_Period = (u16)(65535); //设定计数器自动重装值
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//选择时钟分频：不分频
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM向上计数  
		TIM_TimeBaseInit(timer, &TIM_TimeBaseStructure);
		TIM_EncoderInterfaceConfig(timer, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
		TIM_ICStructInit(&TIM_ICInitStructure);
		TIM_ICInitStructure.TIM_ICFilter = 10;
		TIM_ICInit(timer, &TIM_ICInitStructure);
		TIM_ClearFlag(timer, TIM_FLAG_Update);//清除TIM的更新标志位
		TIM_ITConfig(timer, TIM_IT_Update, ENABLE);
	}
	void begin()
	{
		pinA->mode(INPUT);
		pinB->mode(INPUT);

		//Reset counter
		TIM_SetCounter(TIM2, 0);
		TIM_Cmd(TIM2, ENABLE);
	}
	short getPos()
	{
		pos = timer->CNT;
		return pos;
	}
	void resetPos()
	{
		timer->CNT = 0;
	}
	void refreshDiff()
	{
		getPos();
		diff = pos - posOld;
		posOld = pos;
	}
	short getDiff()
	{
		return diff;
	}
};