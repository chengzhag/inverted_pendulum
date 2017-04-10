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
#include "encoder_exti.h"
#include "freertos.h"
#include "task.h"
#include "queue.h"
#include "led.h"

EncoderExti encoder1(&PA6, &PA7);
Led led1(&PA8,1);

long long oldPos = 0, nowPos = 0;
static void vLEDTask(void *pvParameters)
{
	while (1)
	{
		led1.toggle();
		vTaskDelay(100 / portTICK_RATE_MS);
		nowPos = encoder1.getPosition();
		uart1.printf("%ld\r\n", nowPos/*- oldPos*/);
		oldPos = nowPos;
	}
}

void setup()
{
    ebox_init();
    uart1.begin(115200);
	led1.begin();

	set_systick_user_event_per_sec(configTICK_RATE_HZ);
	attach_systick_user_event(xPortSysTickHandler);

	xTaskCreate(vLEDTask, "LED0", configMINIMAL_STACK_SIZE, (void *)0, NULL, NULL);
	vTaskStartScheduler();
}

int main(void)
{
    setup();

    while(1)
    {

    }
}


