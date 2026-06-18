#ifndef TASK_CREATE_H
#define TASK_CREATE_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* ======================== 任务配置宏 ======================== */

/* 开始任务 - 系统初始化后删除 */
#define START_TASK_STACK_SIZE    256
#define START_TASK_PRIORITY      32

/* 主控任务 - 核心控制逻辑 */
#define MAIN_TASK_STACK_SIZE     2048
#define MAIN_TASK_PRIORITY       12

/* 正弦波生成任务 */
#define SIN_TASK_STACK_SIZE      512
#define SIN_TASK_PRIORITY        12

/* 电机控制任务 */
#define MOTOR_TASK_STACK_SIZE    1024
#define MOTOR_TASK_PRIORITY      10

/* ======================== 任务句柄 ======================== */

extern TaskHandle_t Start_handler;
extern TaskHandle_t main_handler;
extern TaskHandle_t sin_handler;
extern TaskHandle_t motor_handler;

/* ======================== 任务函数声明 ======================== */

void Start_task(void *pvParameters);
void main_task(void *pvParameters);
void sin_task(void *pvParameters);
void motor_task(void *pvParameters);

/* ======================== 任务创建函数 ======================== */

/**
 * @brief 通用任务创建函数
 * @param  func           任务函数指针
 * @param  name           任务名称
 * @param  stack_size     任务堆栈大小
 * @param  priority       任务优先级
 * @param  handle         任务句柄指针
 * @return BaseType_t     pdPASS 创建成功, pdFAIL 创建失败
 */
BaseType_t create_task(TaskFunction_t func, const char *name,
                       uint32_t stack_size, UBaseType_t priority,
                       TaskHandle_t *handle);

void Start_task_create(void);
void main_task_create(void);
void SIN_task_create(void);
void motor_task_create(void);

#endif /* TASK_CREATE_H */
