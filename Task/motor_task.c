/**
 * @file    motor_task.c
 * @brief   电机控制任务模块
 * @details 包含电机 PID 控制、模式切换、编码器读取等功能
 */

#include "motor_task.h"
#include "encoder.h"
#include "motor.h"
#include "uart.h"
#include "speed_ctrl.h"
#include "pid.h"
#include "turn.h"
#include "scaner.h"
#include "bsp_linefollower.h"
#include "sin_generate.h"
#include "map.h"
#include "chassis_api.h"
#include "delay.h"
#include "debug_uart.h"
#include "math.h"
#include "barrier.h"

/* ======================== 速度 PID 参数查表结构 ======================== */

typedef struct {
    float min_measure;      /* 编码器测量值下限 */
    float max_measure;      /* 编码器测量值上限 */
    float kp;               /* 比例系数 */
    float kd;               /* 微分系数 */
} SpeedPidParam;

/* ======================== 私有变量 ======================== */

static int dirct[4] = {-1, 1, -1, -1};     /* 电机方向数组 */
static uint8_t line_gyro_switch = 0;        /* 循线/陀螺仪切换标志 */
static float speed_raw[4] = {0};            /* 原始编码器速度（用于距离计算） */
static float speed_filtered[4] = {0};       /* 滤波后编码器速度（用于PID） */
static struct Gradual motor_target_L = {0, 0, 0};   /* 左电机目标渐变 */
static struct Gradual motor_target_R = {0, 0, 0};   /* 右电机目标渐变 */

#define ENCODER_FILTER_ALPHA    0.8f        /* 编码器低通滤波系数 */

/* ======================== 全局变量定义 ======================== */

volatile uint8_t PIDMode = is_Free;         /* 当前 PID 模式 */
uint8_t Nosmall = 1;                        /* 小车标志 */
int QQB_num;                                /* QQ 编号 */
int MOTOR_PWM_MAX = 9800;                   /* 电机 PWM 最大值 */

/* 速度变化 / 转弯标志 */
uint8_t change_speed = 0;
uint8_t turnflag = 0;
uint8_t Turn360_Flag = 0;

/* 速度渐变量（struct Gradual 定义见 motor_task.h） */
struct Gradual TC_speed = {0};              /* 循线速度渐变 */
struct Gradual TG_speed = {0};              /* 陀螺仪速度渐变 */

/* 流水灯 / 波浪板标志（上层障碍流程使用，底盘默认关闭） */
uint8_t DownLiuShui = 0;
float   LiuShuiRate = 1.0f;
uint8_t WavePlateLeft_Flag = 0;
uint8_t WavePlateRight_Flag = 0;

/* ======================== 速度 PID 查表 ======================== */

/* change_speed == 1: 高速段 (90~110) */
static const SpeedPidParam speed_pid_level1[] = {
    {90.0f, 110.0f, 11.0f, 17.0f},
};

/* change_speed == 2: 中高速段 (190~210) */
static const SpeedPidParam speed_pid_level2[] = {
    {190.0f, 210.0f, 4.0f, 22.0f},
};

/* change_speed == 3: 多段检测 */
static const SpeedPidParam speed_pid_level3[] = {
    {40.0f,  60.0f,  22.0f, 16.0f},    /* 低速段 */
    {90.0f,  110.0f, 11.0f, 17.0f},    /* 中速段 */
    {190.0f, 210.0f,  4.0f, 22.0f},    /* 高速段 */
};

/* change_speed == 4: 超高速段 (290~310) */
static const SpeedPidParam speed_pid_level4[] = {
    {290.0f, 310.0f, 2.4f, 26.0f},
};

/* ======================== 私有函数声明 ======================== */

static void motor_update_sensors(void);
static void motor_update_pid_mode(void);
static void motor_update_targets(void);
static void motor_apply_pid(void);
static void set_motor_target(float left_scale, float right_scale);
static int check_speed_range(float measure, float min_val, float max_val);
static void apply_speed_pid_params(const SpeedPidParam *table, int count);

/* ======================== 辅助函数实现 ======================== */

/**
 * @brief  检测编码器值是否在指定范围内
 * @param  measure 编码器测量值
 * @param  min_val 范围下限
 * @param  max_val 范围上限
 * @return int     1 在范围内, 0 不在
 */
static int check_speed_range(float measure, float min_val, float max_val)
{
    return (measure > min_val && measure < max_val) ? 1 : 0;
}

/**
 * @brief  应用速度 PID 参数查表结果
 * @param  table 参数表指针
 * @param  count 表项数量
 */
static void apply_speed_pid_params(const SpeedPidParam *table, int count)
{
    for (int i = 0; i < count; i++) {
        if (check_speed_range(motor_L0.measure, table[i].min_measure, table[i].max_measure) ||
            check_speed_range(motor_L1.measure, table[i].min_measure, table[i].max_measure) ||
            check_speed_range(motor_R0.measure, table[i].min_measure, table[i].max_measure) ||
            check_speed_range(motor_R1.measure, table[i].min_measure, table[i].max_measure))
        {
            line_pid_param.kp = table[i].kp;
            line_pid_param.ki = 0;
            line_pid_param.kd = table[i].kd;
            change_speed = 0;
            return;
        }
    }
}

/**
 * @brief  统一设置电机目标速度
 * @param  left_scale  左电机缩放系数
 * @param  right_scale 右电机缩放系数
 */
static void set_motor_target(float left_scale, float right_scale)
{
    float lt = motor_all.Lspeed * left_scale;
    float rt = motor_all.Rspeed * right_scale;
    float up   = motor_all.Cincrement > 0 ? motor_all.Cincrement * 0.5f : 6.0f;
    float down = motor_all.CDOWNincrement > 0 ? motor_all.CDOWNincrement * 0.5f : 5.0f;

    gradual_cal(&motor_target_L, lt, up, down);
    gradual_cal(&motor_target_R, rt, up, down);

    motor_L0.target = motor_target_L.Now;
    motor_L1.target = motor_target_L.Now;
    motor_R0.target = motor_target_R.Now;
    motor_R1.target = motor_target_R.Now;
}

/* ======================== 子函数实现 ======================== */

/**
 * @brief  传感器数据更新
 * @details 读取循线传感器、编码器、红外传感器数据
 */
static void motor_update_sensors(void)
{
    /* 循线模式下获取循线误差值 */
    if (PIDMode == is_Line)
        getline_error();

    /* 获取编码器速度和路程 */
    get_motor_speed();

    /*
     * 距离计算只用左轮编码器（原始值，不乘补偿系数）
     * 原因：右轮编码器有较大补偿系数（R1=1.68），会导致距离不准
     * 速度控制仍用补偿后的值（motor_*.measure）
     */
    float avg_left = (speed_raw[0] + speed_raw[1]) / 2.0f;
    motor_all.Distance += (((avg_left * 10.4f * PI) / 5720.0f) / 0.362f) * MOTOR_DISTANCE_SCALE;

    /* 红外传感器数据更新 */
    if (infrare_open)
        get_Infrared();
}

/**
 * @brief  PID 模式切换处理
 * @details 处理循线/陀螺仪模式之间的平滑切换
 */
static void motor_update_pid_mode(void)
{
    if (line_gyro_switch == 1)
    {
        /* 陀螺仪 -> 循线 切换 */
        TC_speed = TG_speed;
        gyroG_pid = (struct P_pid_obj){0, 0, 0, 0, 0, 0, 0};
        TG_speed = (struct Gradual){0, 0, 0};
        line_gyro_switch = 0;
    }
    else if (line_gyro_switch == 2)
    {
        /* 循线 -> 陀螺仪 切换 */
        TG_speed = TC_speed;
        line_pid_obj = (struct P_pid_obj){0, 0, 0, 0, 0, 0, 0};
        TC_speed = (struct Gradual){0, 0, 0};
        line_gyro_switch = 0;
    }
    else
    {
        /* 循线模式 */
        if (PIDMode == is_Line)
        {
            gradual_cal(&TC_speed, motor_all.Cspeed, motor_all.Cincrement, motor_all.CDOWNincrement);
            Go_Line(TC_speed.Now);
        }
        else
            motor_all.Cspeed = 0;

        /* 转弯模式 */
        if (PIDMode == is_Turn)
        {
            if (Turn360_Flag)
                Turn360Step();
            else if (nodesr.nowNode.function == UpStage ||
                     nodesr.nowNode.function == BSoutPole ||
                     nodesr.nowNode.function == BHM)
            {
                if (Stage_turn_Angle(angle.AngleT))
                    gyroT_pid = (struct P_pid_obj){0, 0, 0, 0, 0, 0};
            }
            else if (Turn_Angle(angle.AngleT))
                gyroT_pid = (struct P_pid_obj){0, 0, 0, 0, 0, 0};
        }

        /* 陀螺仪模式 */
        if (PIDMode == is_Gyro)
        {
            gradual_cal(&TG_speed, motor_all.Gspeed, motor_all.Gincrement, motor_all.GDOWNincrement);
            runWithAngle(angle.AngleG, TG_speed.Now);
        }
        else
            motor_all.Gspeed = 0;
    }
}

/**
 * @brief  电机目标速度计算
 * @details 根据当前模式和状态计算各电机的目标速度
 */
static void motor_update_targets(void)
{
    /* 流水灯模式 */
    if (DownLiuShui)
    {
        set_motor_target(LiuShuiRate, 1.6f);
    }
    /* 左波浪板模式 */
    else if (WavePlateLeft_Flag)
    {
        motor_L0.target = motor_L1.target = motor_all.Lspeed * 1.2f;
        motor_R0.target = motor_R1.target = motor_all.Rspeed;
    }
    /* 右波浪板模式 */
    else if (WavePlateRight_Flag)
    {
        motor_L0.target = motor_L1.target = motor_all.Lspeed;
        motor_R0.target = motor_R1.target = motor_all.Rspeed * 1.2f;
    }
    else
    {
        /* 特殊速度模式 (Cspeed == 50) */
        if (motor_all.Cspeed == 50)
        {
            float left_scale = 1.0f;
            if (QQB_num == 1)
                left_scale = 2.5f;
            else if (QQB_num == 2)
                left_scale = 4.0f;
            set_motor_target(left_scale, 1.0f);
        }
        /* 正常速度模式 */
        else
        {
            set_motor_target(1.0f, 1.0f);
            if (turnflag == 1)
                turnflag = 0;
        }

        /* 速度变化时的 PID 参数调整（查表法） */
        switch (change_speed)
        {
            case 1:
                apply_speed_pid_params(speed_pid_level1, 1);
                break;
            case 2:
                apply_speed_pid_params(speed_pid_level2, 1);
                break;
            case 3:
                apply_speed_pid_params(speed_pid_level3, 3);
                break;
            case 4:
                apply_speed_pid_params(speed_pid_level4, 1);
                break;
            default:
                break;
        }
    }
}

/**
 * @brief  PID 计算和 PWM 输出
 * @details 根据当前模式执行 PID 运算并输出 PWM
 */
static void motor_apply_pid(void)
{
    if (PIDMode != is_Free)
    {
        /* 增量式 PID 计算 */
        incremental_PID(&motor_L0, &motor_pid_paramL0);
        incremental_PID(&motor_L1, &motor_pid_paramL1);
        incremental_PID(&motor_R0, &motor_pid_paramR0);
        incremental_PID(&motor_R1, &motor_pid_paramR1);

        /* 设置电机 PWM 输出 */
        motor_set_pwm(1, (int32_t)motor_L0.output);
        motor_set_pwm(2, (int32_t)motor_L1.output);
        motor_set_pwm(3, (int32_t)motor_R0.output);
        motor_set_pwm(4, (int32_t)motor_R1.output);
    }
}

/* ======================== 电机任务主函数 ======================== */

void motor_task(void *pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        /* 1. 传感器数据更新 */
        motor_update_sensors();

        /* 绝对休眠 5ms */
        vTaskDelayUntil(&xLastWakeTime, (5 / portTICK_RATE_MS));

        /* 2. PID 模式切换处理 */
        motor_update_pid_mode();

        /* 2.5 底盘周期更新（游龙防护 / 丢线保护） */
        Chassis_Periodic_Update_5ms();

        /* 调试串口输出 */
        debug_uart_tick();

        /* 3. 电机目标速度计算 */
        motor_update_targets();

        /* 4. PID 计算和 PWM 输出 */
        motor_apply_pid();
    }
}

/* ======================== PID 模式切换函数 ======================== */

void pid_mode_switch(uint8_t target_mode)
{
    if (Chassis_IsTipoverLocked() && target_mode != is_No)
        target_mode = is_No;

    switch (target_mode)
    {
        case is_Turn:
        {
            MOTOR_PWM_MAX = 5000;
            line_pid_obj = (struct P_pid_obj){0, 0, 0, 0, 0, 0, 0};
            gyroG_pid = (struct P_pid_obj){0, 0, 0, 0, 0, 0, 0};
            gyroT_pid = (struct P_pid_obj){0, 0, 0, 0, 0, 0, 0};
            TC_speed = (struct Gradual){0, 0, 0};
            TG_speed = (struct Gradual){0, 0, 0};
            motor_target_L = (struct Gradual){0, 0, 0};
            motor_target_R = (struct Gradual){0, 0, 0};
            break;
        }

        case is_Line:
        {
            MOTOR_PWM_MAX = 9800;
            if (PIDMode == is_Gyro)
                line_gyro_switch = 1;   /* 陀螺仪 -> 循线 */
            else
                gyroT_pid = (struct P_pid_obj){0, 0, 0, 0, 0, 0, 0};
            break;
        }

        case is_Gyro:
        {
            MOTOR_PWM_MAX = 9800;
            if (PIDMode == is_Line)
                line_gyro_switch = 2;   /* 循线 -> 陀螺仪 */
            else
                gyroT_pid = (struct P_pid_obj){0, 0, 0, 0, 0, 0, 0};
            break;
        }

        case is_Free:
        {
            MOTOR_PWM_MAX = 9800;
            break;
        }

        case is_No:
        {
            MOTOR_PWM_MAX = 9800;
            motor_all.Lspeed = 0;
            motor_all.Rspeed = 0;
            motor_all.Cspeed = 0;
            motor_all.Gspeed = 0;
            motor_L0.target = 0;
            motor_L1.target = 0;
            motor_R0.target = 0;
            motor_R1.target = 0;
            line_pid_obj = (struct P_pid_obj){0, 0, 0, 0, 0, 0, 0};
            gyroT_pid = (struct P_pid_obj){0, 0, 0, 0, 0, 0, 0};
            gyroG_pid = (struct P_pid_obj){0, 0, 0, 0, 0, 0, 0};
            TG_speed = (struct Gradual){0, 0, 0};
            TC_speed = (struct Gradual){0, 0, 0};
            motor_target_L = (struct Gradual){0, 0, 0};
            motor_target_R = (struct Gradual){0, 0, 0};
            motor_pid_clear();
            for (int i = 0; i < 4; i++) speed_filtered[i] = 0;
            motor_set_pwm(1, 0);
            motor_set_pwm(2, 0);
            motor_set_pwm(3, 0);
            motor_set_pwm(4, 0);
            break;
        }
    }

    PIDMode = target_mode;
}

void pid_mode_switch_no_inherit(uint8_t target_mode)
{
    pid_mode_switch(target_mode);
    line_gyro_switch = 0;
    line_pid_obj = (struct P_pid_obj){0, 0, 0, 0, 0, 0, 0};
    gyroG_pid = (struct P_pid_obj){0, 0, 0, 0, 0, 0, 0};
    TG_speed = (struct Gradual){0, 0, 0};
    TC_speed = (struct Gradual){0, 0, 0};
}

/* ======================== 编码器读取函数 ======================== */

void get_motor_speed(void)
{
    /* 读取四个电机的编码器计数值 */
    Speed[0] = (short)TIM1->CNT;   /* 左前 */
    TIM1->CNT = 0;

    Speed[1] = (short)TIM2->CNT;   /* 左后 */
    TIM2->CNT = 0;

    Speed[2] = (short)TIM3->CNT;   /* 右前 */
    TIM3->CNT = 0;

    Speed[3] = (short)TIM5->CNT;   /* 右后 */
    TIM5->CNT = 0;

    /* 设置电机方向 */
    dirct[0] = dirct[1] = 1;
    dirct[2] = dirct[3] = -1;

    /* 保存原始编码器值（用于距离计算，不乘补偿系数） */
    speed_raw[0] = (float)Speed[0] * dirct[0];
    speed_raw[1] = (float)Speed[1] * dirct[1];
    speed_raw[2] = (float)Speed[2] * dirct[2];
    speed_raw[3] = (float)Speed[3] * dirct[3];

    /* 一阶低通滤波：抑制编码器噪声尖峰 */
    speed_filtered[0] += ENCODER_FILTER_ALPHA * (speed_raw[0] - speed_filtered[0]);
    speed_filtered[1] += ENCODER_FILTER_ALPHA * (speed_raw[1] - speed_filtered[1]);
    speed_filtered[2] += ENCODER_FILTER_ALPHA * (speed_raw[2] - speed_filtered[2]);
    speed_filtered[3] += ENCODER_FILTER_ALPHA * (speed_raw[3] - speed_filtered[3]);

    /* 计算各电机实际速度（补偿后，用于PID控制） */
    motor_L0.measure = speed_filtered[0];
    motor_L1.measure = speed_filtered[1];
    motor_R0.measure = speed_filtered[2] * R0_ENCODER_SCALE;
    motor_R1.measure = speed_filtered[3] /** R1_ENCODER_SCALE*/;
}
