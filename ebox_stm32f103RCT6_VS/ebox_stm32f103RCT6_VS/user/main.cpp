/**
  ******************************************************************************
  * @file   : *.cpp
  * @author : shentq
  * @version: V1.2
  * @date   : 2016/08/14

  * @brief   ebox application example .
  *
  * Copyright 2016 shentq. All Rights Reserved.         
  ******************************************************************************
 */

#include "ebox.h"
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "led.h"
#include "encoder_timer.h"
#include "encoder_motor.h"
#include "tb6612fng.h"
#include "PID.hpp"

Led led1(&PC13, 1);
 //未重映射的情况下
 //- TIM1 : PA8 PA9
 //- TIM2 : PA0 PA1
 //- TIM3 : PA6 PA7
 //- TIM4 : PB6 PB7
EncoderTimer encoder(TIM4);
EncoderMotor motor(TIM3, &PA2, &PA1, &PA0);
greg::PID pendulumPID,posPID;


static void vDebugTask(void *pvParameters)
{
	while (1)
	{
		led1.toggle();
		vTaskDelay(100 / portTICK_RATE_MS);
		uart1.printf("%f\t\t%ld\t\t%d\t\t%ld\r\n", 
			motor.getPercent(),
			motor.getSpd(), 
			motor.getPos(),
			encoder.getPos()
			);
	}
}

long posMotor = 0;
long posPendulum = 0;
static void vPIDTask(void *pvParameters)
{
	static long desiredPosPendulum = 0;
	while (1)
	{
		vTaskDelay(10 / portTICK_RATE_MS);
		
		//刷新编码器和电机位置PID
		encoder.refresh();
		motor.refresh();
		posMotor = motor.getPos();

		//横梁位置
		desiredPosPendulum=posPID.refresh(-posMotor);
		//摆杆角度PID
		posPendulum = -encoder.getPos();
		pendulumPID.setDesiredPoint(desiredPosPendulum);
		if (posPendulum<200 && posPendulum>-200)
			motor.setPosDiff(pendulumPID.refresh(posPendulum));
	}
}


void setup()
{
    ebox_init();
    uart1.begin(115200);
	led1.begin();
	encoder.begin();
	motor.begin(1.5, 0.1, 0.02);

	//motor1.setPos(200);

	//初始化摆杆角度PID
	pendulumPID.setRefreshInterval(0.01);
	pendulumPID.setWeights(0.3, 0.5, 0.02);
	pendulumPID.setOutputLowerLimit(-50);
	pendulumPID.setOutputUpperLimit(50);
	pendulumPID.setDesiredPoint(0);

	//初始化衡量位置PID
	posPID.setRefreshInterval(0.01);
	posPID.setWeights(0.1, 0, 0);
	posPID.setOutputLowerLimit(-100);
	posPID.setOutputUpperLimit(100);
	posPID.setDesiredPoint(0);

	//设置RTOS进程
	set_systick_user_event_per_sec(configTICK_RATE_HZ);
	attach_systick_user_event(xPortSysTickHandler);

	xTaskCreate(vDebugTask, "Debug", configMINIMAL_STACK_SIZE, (void *)0, NULL, NULL);
	xTaskCreate(vPIDTask, "PID", configMINIMAL_STACK_SIZE, (void *)0, NULL, NULL);
	vTaskStartScheduler();
}

int main(void)
{
    setup();

    while(1)
    {
    }
}


