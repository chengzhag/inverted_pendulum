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
#include "inverted_pendulum.h"

#define PID_REFRESH_INTERVAL 0.01


Led led1(&PC13, 1);

InvertedPendulum invertedPendulum(TIM4, TIM3,
	&PA2, &PA1, &PA0,
	2000, 1560, PID_REFRESH_INTERVAL);

//用于观察波形的全局变量
//float motorRadian = 0;
//float pendulumRadian = 0;

static void vDebugTask(void *pvParameters)
{
	while (1)
	{
		led1.toggle();
		vTaskDelay(20 / portTICK_RATE_MS);
		uart1.printf("%f\t%f\t%f\t%f\t%f\r\n",
			invertedPendulum.getBeamRadian(),
			invertedPendulum.getBeamPalstance(),
			invertedPendulum.getPendulumRadian(),
			invertedPendulum.getPendulumPalstance(),
			invertedPendulum.getPendulumAcceleration()
		);

		//用于观察波形的全局变量
		//motorRadian = invertedPendulum.getBeamRadian();
		//pendulumRadian= invertedPendulum.getPendulumRadian();
	}
}


static void vPIDTask(void *pvParameters)
{
	while (1)
	{
		vTaskDelay(PID_REFRESH_INTERVAL*1000 / portTICK_RATE_MS);
		invertedPendulum.refresh();
	}
}


void setup()
{
    ebox_init();
    uart1.begin(115200);
	led1.begin();

	invertedPendulum.begin();
	invertedPendulum.setMode(Inverted_Pendulum_Mode_Round);
	invertedPendulum.setTargetBeamPalstance(2);

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


