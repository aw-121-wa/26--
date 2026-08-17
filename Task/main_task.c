/**
 * @file    main_task.c
 * @brief   主控任务模块
 * @details 主任务循环：调用zhunbei()准备，然后循环调用Cross()进行节点间处理
 */

#include "main_task.h"
#include "../App/map/map.h"
#include "../App/barrier/barrier.h"
#include "../App/chassis/chassis_api.h"
#include "../App/vision/vision_api.h"
#include "../Sensor/bsp_linefollower.h"
#include "motor_task.h"
#include "encoder.h"

/* 测试模式：起点设为P3，跳过前面路线。置1启用，置0恢复默认。 */
#define TEST_START_P3   0

/* 测试模式：起点设为N22，挡板检测后直接向B6走。置1启用。 */
#define TEST_START_N22_B6   0

/* 测试模式：起点设为N22，挡板检测后向C10出发，后续与主路线后半段一致。置1启用。 */
#define TEST_START_N22_C10  1  

/**
 * @brief  主任务函数
 * @details 执行流程：
 *          1. 调用zhunbei()准备（下坡+等待挡板）
 *          2. 循环调用Cross()进行节点间处理
 *          3. 当map.routetime递增时表示一轮结束
 */
void main_task(void *pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

#if TEST_START_N22_C10
    /* --- N22→C10测试模式：起点N22，挡板检测后向C10出发，后续与主路线后半段一致 --- */
    mapInit_test_N22_C10();

    /* 红外挡板检测（与zhunbei一致，但跳过P2下坡） */
    Chassis_SetMode(is_No);
    motor_all.Lspeed = 0;
    motor_all.Rspeed = 0;
    infrare_open = 1;
    vTaskDelay(100);
    while (Infrared_ahead == 0) vTaskDelay(5);   /* 等待挡板 */
    while (Infrared_ahead == 1) vTaskDelay(5);   /* 等待移除挡板 */
    motor_all.Cincrement = 0.5f;   /* smooth accel, same as zhunbei */
    Chassis_SetTargetSpeed(SPEED2);   /* N22→C10段速度SPEED2 */
#elif TEST_START_N22_B6
    /* --- N22测试模式：起点N22，挡板检测后直接向B6走 --- */
    mapInit_test_N22_B6();

    /* 红外挡板检测（与zhunbei一致，但跳过P2下坡） */
    Chassis_SetMode(is_No);
    motor_all.Lspeed = 0;
    motor_all.Rspeed = 0;
    infrare_open = 1;
    vTaskDelay(100);
    while (Infrared_ahead == 0) vTaskDelay(5);   /* 等待挡板 */
    while (Infrared_ahead == 1) vTaskDelay(5);   /* 等待移除挡板 */
    Chassis_SetTargetSpeed(SPEED1);   /* N22→B6段速度SPEED1 */
#elif TEST_START_P3
    /* --- P3测试模式：起点P3，挡板检测后执行Stage原地转，之后P3→N3→N8 --- */
    mapInit_test_P3();

    /* 红外挡板检测（与zhunbei一致，但跳过P2下坡） */
    Chassis_SetMode(is_No);
    motor_all.Lspeed = 0;
    motor_all.Rspeed = 0;
    infrare_open = 1;
    vTaskDelay(100);
    while (Infrared_ahead == 0) vTaskDelay(5);   /* 等待挡板 */
    while (Infrared_ahead == 1) vTaskDelay(5);   /* 等待移除挡板 */
    Chassis_SetTargetSpeed(SPEED4);
#else
    /* 地图初始化 */
    Vision_Init();
    mapInit();

    /* 准备流程：下坡、等待挡板、切换循线 */
    zhunbei();
#endif

#if LINE_DEBUG_MODE
    map.routetime = 2;  /* 跳过Cross，纯巡线调PID */
#endif

    /* 清零里程 */
    encoder_clear();

    /* 主循环 */
    while (1)
    {
        /* 节点间处理 */
        if (map.routetime == 0)
            Cross();

        /* 一轮结束处理 */
        if (map.routetime == 1)
        {
            Chassis_SetMode(is_No);

            /* 第二轮流程当前未启用，保持停车状态。 */
            map.routetime = 2;
        }

        /* 绝对休眠 5ms */
        vTaskDelayUntil(&xLastWakeTime, (5 / portTICK_RATE_MS));
    }
}
