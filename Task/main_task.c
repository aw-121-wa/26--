/**
 * @file    main_task.c
 * @brief   主控任务模块
 * @details 主任务循环：调用zhunbei()准备，然后循环调用Cross()进行节点间处理
 */

#include "main_task.h"
#include "../App/map/map.h"
#include "../App/map/route_catalog.h"
#include "../App/map/traffic_route.h"
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
#define TEST_START_N22_C10  0  

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
#if LINE_DEBUG_MODE
        /* 纯巡线调 PID：跳过 Cross 与两轮生命周期，由 motor_task 持续巡线 */
        (void)0;
#else
        /* 节点间处理：第一轮(0)与第二轮(2)都运行 Cross */
        if (map.routetime == 0 || map.routetime == 2)
            Cross();

        /* 第一轮结束：装载第二轮并使用第一轮已确认可通行的门再次出发 */
        if (map.routetime == 1)
        {
            uint8_t gate;
            const uint8_t *round2_route;

            CarBrake();
            Chassis_SetMode(is_No);

            gate = TrafficRoute_GetFirstPassableGate();
            round2_route = RouteCatalog_GetRound2Fast(gate);

            if (round2_route == 0)
            {
                /* 第一轮无确认可通行的门：无法构成第二轮最短路线，永久停车 */
                Chassis_ForceStop(CHASSIS_STOP_ROUTE_INVALID);
            }
            else
            {
                /* 覆盖 route 为二轮路线 + 重建 P2 起点现场（不重置第一轮比赛信息） */
                Map_StartRound2(round2_route);
                zhunbei();
                encoder_clear();
                motor_pid_clear();
            }
        }

        /* 第二轮结束：永久停车 */
        if (map.routetime >= 3)
        {
            CarBrake();
            Chassis_SetMode(is_No);
        }
#endif

        /* 绝对休眠 5ms */
        vTaskDelayUntil(&xLastWakeTime, (5 / portTICK_RATE_MS));
    }
}
