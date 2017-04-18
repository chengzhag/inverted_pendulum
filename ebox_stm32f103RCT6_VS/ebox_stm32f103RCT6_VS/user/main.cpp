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

#define PID_REFRESH_INTERVAL 0.005

Led led1(&PC13, 1);

EncoderTimer encoder(TIM4);
EncoderMotor motor(TIM3, &PA2, &PA1, &PA0,Encoder_Motor_Target_Position, PID_REFRESH_INTERVAL);
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
		vTaskDelay(PID_REFRESH_INTERVAL*1000 / portTICK_RATE_MS);
		
		//刷新编码器和电机位置PID
		encoder.refresh();
		motor.refresh();
		posMotor = motor.getPos();

		posPendulum = -encoder.getPos();
		if (posPendulum<400 && posPendulum>-400)
		{
			//横梁位置
			desiredPosPendulum = posPID.refresh(-posMotor);
			pendulumPID.setDesiredPoint(desiredPosPendulum);
			//摆杆角度PID
			motor.setPosDiff(pendulumPID.refresh(posPendulum));
		}
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

	//初始化摆杆角度PID
	pendulumPID.setRefreshInterval(PID_REFRESH_INTERVAL);
	pendulumPID.setWeights(0.4, 1.05, 0.0005);
	pendulumPID.setOutputLowerLimit(-100);
	pendulumPID.setOutputUpperLimit(100);
	pendulumPID.setDesiredPoint(0);

	//初始化衡量位置PID
	posPID.setRefreshInterval(PID_REFRESH_INTERVAL);
	posPID.setWeights(0.04, 0, 0.045);
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


