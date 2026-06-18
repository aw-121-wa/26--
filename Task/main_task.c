/**
 * @file    main_task.c
 * @brief   主控任务模块
 * @details 主任务循环：调用zhunbei()准备，然后循环调用Cross()进行节点间处理
 */

#include "main_task.h"
#include "../App/map/map.h"
#include "../App/barrier/barrier.h"
#include "../App/chassis/chassis_api.h"
#include "motor_task.h"
#include "encoder.h"
#include "imu.h"
#include "delay.h"

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

    /* 地图初始化 */
    mapInit();

    /* 准备流程：下坡、等待挡板、切换循线 */
    zhunbei();

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
            /* 停车 */
            pid_mode_switch(is_No);

            /* TODO: 添加第二轮处理逻辑 */
            /* mapInit1();
             * zhunbei();
             * encoder_clear();
             * pid_mode_switch(is_Line);
             * motor_all.Cspeed = SPEED1;
             */

            map.routetime = 2;
        }

        /* 绝对休眠 5ms */
        vTaskDelayUntil(&xLastWakeTime, (5 / portTICK_RATE_MS));
    }
}
