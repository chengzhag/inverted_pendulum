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

Led led1(&PC13, 1);
 //未重映射的情况下
 //- TIM1 : PA8 PA9
 //- TIM2 : PA0 PA1
 //- TIM3 : PA6 PA7
 //- TIM4 : PB6 PB7
EncoderTimer encoder(TIM4);
EncoderMotor motor(TIM3, &PA2, &PA1, &PA0);


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

static void vPIDTask(void *pvParameters)
{
	while (1)
	{
		vTaskDelay(10 / portTICK_RATE_MS);
		encoder.refresh();
		motor.refresh();
		posMotor = motor.getPos();
	}
}


void setup()
{
    ebox_init();
    uart1.begin(115200);
	led1.begin();
	encoder.begin();
	motor.begin();

	//motor1.setPos(200);

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


