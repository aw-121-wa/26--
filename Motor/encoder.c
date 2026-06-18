/**
 * @file    Encoder.c
 * @brief   电机编码器驱动模块
 * @details 通过定时器编码器模式读取四个电机的转速
 */

#include "encoder.h"
#include "motor_task.h"

/* ======================== 全局变量定义 ======================== */

short Speed[4];                     /* 四个电机的编码器速度 */
uint8_t dog[4] = {0};              /* 电机看门狗 */

/* ======================== 编码器初始化 ======================== */

/**
 * @brief  编码器初始化
 * @details 启动四个电机的编码器定时器和辅助定时器：
 *          - TIM1: 左前电机
 *          - TIM2: 左后电机
 *          - TIM3: 右前电机
 *          - TIM5: 右后电机
 *          - TIM11: 辅助定时器
 */
void Encoder_init(void)
{
    /* 左前电机编码器 */
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_Base_Start_IT(&htim1);

    /* 左后电机编码器 */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_Base_Start_IT(&htim2);

    /* 右前电机编码器 */
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_Base_Start_IT(&htim3);

    /* 右后电机编码器 */
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_Base_Start_IT(&htim5);

    /* 辅助定时器 */
    HAL_TIM_Base_Start_IT(&htim11);
}

/* ======================== 定时器回调函数 ======================== */

/* HAL_TIM_PeriodElapsedCallback 由 Core/Src/main.c 统一实现（TIM14 -> HAL_IncTick） */

/* ======================== 编码器清零 ======================== */

/**
 * @brief  清零编码器计数和里程
 */
void encoder_clear(void)
{
    motor_all.Distance = 0;

    TIM1->CNT = 0;
    TIM2->CNT = 0;
    TIM3->CNT = 0;
    TIM5->CNT = 0;
}
