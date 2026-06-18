/**
 * @file    motor.c
 * @brief   电机驱动模块
 * @details 控制四个电机的 PWM 输出和方向
 */

#include "motor.h"
#include "motor_task.h"

/*
 * 电机引脚映射：
 * - R0: TIM4.3 (PD14), TIM4.4 (PD15)
 * - R1: TIM4.1 (PD12), TIM4.2 (PD13)
 * - L0: TIM8.3 (PC8),  TIM8.4 (PC9)
 * - L1: TIM9.1 (PE5),  TIM9.2 (PE6)
 */

/* ======================== 电机初始化 ======================== */

/**
 * @brief  电机和 PID 初始化
 */
void motor_init(void)
{
    /* 启动定时器 */
    HAL_TIM_Base_Start(&htim4);
    HAL_TIM_Base_Start(&htim8);
    HAL_TIM_Base_Start(&htim9);

    /* R1 电机 (TIM4 通道 1/2) */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

    /* R0 电机 (TIM4 通道 3/4) */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    /* L0 电机 (TIM8 通道 3/4) */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

    /* L1 电机 (TIM9 通道 1/2) */
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);

    /* 初始化 PID 参数 */
    pid_init();
}

/* ======================== PWM 输出控制 ======================== */

/**
 * @brief  设置电机 PWM 输出
 * @param  motor   电机编号 (1=L0, 2=L1, 3=R0, 4=R1)
 * @param  pid_out PID 输出值（正负控制方向，幅值控制占空比）
 */
void motor_set_pwm(uint8_t motor, int32_t pid_out)
{
    int32_t ccr = 0;

    if (pid_out >= 0)
    {
        /* 正转 */
        ccr = (pid_out > MOTOR_PWM_MAX) ? MOTOR_PWM_MAX : pid_out;

        switch (motor)
        {
            case 1: TIM8->CCR3 = 0;    TIM8->CCR4 = ccr;  break;  /* L0 */
            case 2: TIM9->CCR2 = 0;    TIM9->CCR1 = ccr;  break;  /* L1 */
            case 3: TIM4->CCR3 = 0;    TIM4->CCR4 = ccr;  break;  /* R0 */
            case 4: TIM4->CCR1 = 0;    TIM4->CCR2 = ccr;  break;  /* R1 */
            default: break;
        }
    }
    else
    {
        /* 反转 */
        ccr = (pid_out < -MOTOR_PWM_MAX) ? MOTOR_PWM_MAX : -pid_out;

        switch (motor)
        {
            case 1: TIM8->CCR4 = 0;    TIM8->CCR3 = ccr;  break;  /* L0 */
            case 2: TIM9->CCR1 = 0;    TIM9->CCR2 = ccr;  break;  /* L1 */
            case 3: TIM4->CCR4 = 0;    TIM4->CCR3 = ccr;  break;  /* R0 */
            case 4: TIM4->CCR2 = 0;    TIM4->CCR1 = ccr;  break;  /* R1 */
            default: break;
        }
    }
}
