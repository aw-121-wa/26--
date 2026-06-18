#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include "pid.h"
#include "tim.h"

/*
 * 电机 PWM 参数：
 * - PWM 范围：0 ~ MOTOR_PWM_MAX
 * - 速度对应：PWM 10000 ≈ 编码器 200
 */

/**
 * @brief  电机和 PID 初始化
 * @details 启动 TIM4/TIM8/TIM9 的 PWM 输出，并初始化 PID 参数
 */
void motor_init(void);

/**
 * @brief  设置电机 PWM 输出
 * @param  motor   电机编号 (1=L0, 2=L1, 3=R0, 4=R1)
 * @param  pid_out PID 输出值（正负控制方向）
 */
void motor_set_pwm(uint8_t motor, int32_t pid_out);

#endif /* MOTOR_H */
