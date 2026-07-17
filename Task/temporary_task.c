/**
 * @file    temporary_task.c
 * @brief   系统启动与初始化
 * @details Start_task 创建主控/电机任务后自删；user_init 初始化底盘、PCA9685
 *          舵机和 UART5 MaixCam 接口。
 */

#include "temporary_task.h"
#include "task_create.h"
#include "delay.h"
#include "motor.h"
#include "encoder.h"
#include "imu.h"
#include "scaner.h"
#include "map.h"
#include "rudder_control.h"
#include "vision_api.h"
#include "FreeRTOS.h"
#include "task.h"

/* ======================== 开始任务 ======================== */

/**
 * @brief  开始任务：创建主控/电机任务后删除自身
 */
void Start_task(void *pvParameters)
{
    taskENTER_CRITICAL();

    main_task_create();
    motor_task_create();

    taskEXIT_CRITICAL();

    vTaskDelete(NULL);      /* 删除自身（须在退出临界区之后） */
}

/* ======================== 底盘外设初始化 ======================== */

/**
 * @brief  底盘外设初始化
 * @details 顺序：delay → 编码器 → IMU → 电机；随后标定 IMU 角度基准。
 * @note    必须先 delay_init()，否则 delay_us/delay_ms 会因 TIM7 未启动而死等。
 */
void user_init(void)
{
    delay_init();
    Rudder_Init();
    Rudder_control(VISION_SERVO_CENTER, VISION_SERVO_CHANNEL);
    (void)Vision_Init();
    scaner_gpio_init();     /* 16 路循迹灯输入引脚 */
    scaner_init();          /* 循迹权重初始化（line_weight ← line_weight_default） */
    Encoder_init();
    imu_receive_init();
    motor_init();           /* 内部完成 pid_init() */

    delay_ms(500);          /* 等待 IMU 上电稳定 */

    /* IMU 角度基准标定：采样 10 次取平均 */
    float yaw_sum = 0.0f;
    float pitch_sum = 0.0f;
    float roll_sum = 0.0f;
    for (uint8_t i = 0; i < 10; i++)
    {
        delay_ms(5);
        yaw_sum   += imu.yaw;
        pitch_sum += imu.pitch;
        roll_sum  += imu.roll;
    }
    basic_y = yaw_sum / 10.0f;
    basic_p = pitch_sum / 10.0f;
    basic_r = roll_sum / 10.0f;

    mpuZreset(basic_y, nodesr.nowNode.angle);
}
