/*
 * @File sin_generate.c
 * @Description: 
 * @Version: 1.0.0
 * @Author: 
 * @Date: 2023-09-13 20:33:34
 * @LastEditTime: 2023-09-15 15:43:07
 */
#include "sin_generate.h"
#include "math.h"
#include "main.h"
#include "motor_task.h"
#include "motor.h"
#include "pid.h"
#include "barrier.h"
TaskHandle_t sin_task_handler;


struct sin_param sin_use_motor={0,0,100,0.1};//150

struct sin_param sin1={0,0,50,0.1};

void sin_task(void *pvParameters){
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	while(1){
		for(;;)
		{
			motor_L1.target = sin_generator(&sin_use_motor);
			incremental_PID(&motor_L1, &motor_pid_paramL0);
			motor_set_pwm(1, 0);
			motor_set_pwm(2, (int32_t)motor_L1.output);
			motor_set_pwm(3, 0);
			motor_set_pwm(4, 0);
			vTaskDelayUntil(&xLastWakeTime, (5/portTICK_RATE_MS));
		}
	}
}

/**
 * @brief: 
 * @param {sin_param} *param
 * @return {*}
 */
float sin_generator(struct sin_param *param)
{
	float output;
	
	param->actual_t = param->time * param->angular_velocity;
	
	output = param->gain * sin(param->actual_t * PI/180.0f);
	
	++param->time;
	
	if (param->actual_t >= 360)
		param->time = 0;
	
	return output;
}
