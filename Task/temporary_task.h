#ifndef TEMPORARY_TASK_H
#define TEMPORARY_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/**
 * @brief  打印任务剩余栈空间
 * @param  xTask 任务句柄，传 NULL 则打印当前任务
 */
void GET_free_RAM(TaskHandle_t xTask);

/**
 * @brief  系统硬件初始化
 * @details 初始化串口、编码器、IIC、灰度传感器、舵机、IMU、LCD、电机等
 */
void user_init(void);

#endif /* TEMPORARY_TASK_H */
