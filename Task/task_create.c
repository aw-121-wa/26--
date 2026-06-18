/**
 * @file    task_create.c
 * @brief   FreeRTOS 任务创建模块
 * @details 统一的任务创建接口，减少重复代码
 */

#include "task_create.h"

/* ======================== 任务句柄定义 ======================== */

TaskHandle_t Start_handler;     /* 开始任务句柄 */
TaskHandle_t main_handler;      /* 主控任务句柄 */
TaskHandle_t sin_handler;       /* 正弦任务句柄 */
TaskHandle_t motor_handler;     /* 电机任务句柄 */

/* ======================== 通用任务创建 ======================== */

BaseType_t create_task(TaskFunction_t func, const char *name,
                       uint32_t stack_size, UBaseType_t priority,
                       TaskHandle_t *handle)
{
    return xTaskCreate(func, name, stack_size, NULL, priority, handle);
}

/* ======================== 各任务创建函数 ======================== */

void Start_task_create(void)
{
    create_task(Start_task, "Start_task",
                START_TASK_STACK_SIZE, START_TASK_PRIORITY,
                &Start_handler);
}

void main_task_create(void)
{
    create_task(main_task, "main_task",
                MAIN_TASK_STACK_SIZE, MAIN_TASK_PRIORITY,
                &main_handler);
}

void SIN_task_create(void)
{
    create_task(sin_task, "sin_task",
                SIN_TASK_STACK_SIZE, SIN_TASK_PRIORITY,
                &sin_handler);
}

void motor_task_create(void)
{
    create_task(motor_task, "motor_task",
                MOTOR_TASK_STACK_SIZE, MOTOR_TASK_PRIORITY,
                &motor_handler);
}
