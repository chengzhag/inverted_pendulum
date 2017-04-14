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
#include "encoder_exti.h"
#include "encoder_motor.h"
#include "tb6612fng.h"

EncoderExti encoder1(&PA5, &PA6);
EncoderMotor motor1(&PA3, &PA4, &PA1, &PA2, &PA0);

Led led1(&PC13,1);

static void vLEDTask(void *pvParameters)
{
	while (1)
	{
		led1.toggle();
		vTaskDelay(100 / portTICK_RATE_MS);
		uart1.printf("%f\t\t%ld\t\t%ld\t\t%ld\r\n", 
			motor1.getPercent(),
			motor1.getSpd(), 
			motor1.getPos(),
			encoder1.getPos()
			);
	}
}

long pos = 0;
static void vPIDTask(void *pvParameters)
{
	while (1)
	{
		vTaskDelay(10 / portTICK_RATE_MS);
		motor1.refresh();
		pos = motor1.getPos();
	}
}


void setup()
{
    ebox_init();
    uart1.begin(115200);
	led1.begin();
	encoder1.begin();
	motor1.begin();

	motor1.setPos(1000);

	set_systick_user_event_per_sec(configTICK_RATE_HZ);
	attach_systick_user_event(xPortSysTickHandler);

	xTaskCreate(vLEDTask, "LED0", configMINIMAL_STACK_SIZE, (void *)0, NULL, NULL);
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


