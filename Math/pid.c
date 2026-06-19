/**
 * @file    pid.c
 * @brief   PID 控制算法模块
 * @details 包含增量式 PID 和位置式 PID 的实现，以及各通道 PID 参数初始化
 */

#include "pid.h"
#include "motor.h"
#include "stdio.h"
#include "motor_task.h"
#include "sin_generate.h"
#include "math.h"

/* ======================== 电机 PID 对象定义 ======================== */

struct I_pid_obj motor_L0 = {0, 0, 0, 0, 0, 0};    /* 左前电机 */
struct I_pid_obj motor_L1 = {0, 0, 0, 0, 0, 0};    /* 左后电机 */
struct I_pid_obj motor_R0 = {0, 0, 0, 0, 0, 0};    /* 右前电机 */
struct I_pid_obj motor_R1 = {0, 0, 0, 0, 0, 0};    /* 右后电机 */

struct PID_param motor_pid_paramL0;     /* 左前电机 PID 参数 */
struct PID_param motor_pid_paramL1;     /* 左后电机 PID 参数 */
struct PID_param motor_pid_paramR0;     /* 右前电机 PID 参数 */
struct PID_param motor_pid_paramR1;     /* 右后电机 PID 参数 */

/* ======================== 循线 PID 对象定义 ======================== */

struct P_pid_obj line_pid_obj = {0, 0, 0, 0, 0, 0};
struct PID_param line_pid_param;
struct PID_param lineG_pid_param;

/* ======================== 陀螺仪 PID 对象定义 ======================== */

struct P_pid_obj gyroT_pid = {0, 0, 0, 0, 0, 0};   /* 转弯控制 */
struct P_pid_obj gyroG_pid = {0, 0, 0, 0, 0, 0};   /* 平滑陀螺仪 */
struct PID_param gyroT_pid_param;
struct PID_param gyroG_pid_param;

struct P_pid_obj GyroP_pid = {0, 0, 0, 0, 0, 0};   /* 漂移补偿 */
struct PID_param GyroP_pid_param;

/* ======================== 增量式 PID 算法 ======================== */

/**
 * @brief  增量式 PID 计算
 * @param  motor 电机 PID 对象指针
 * @param  pid   PID 参数指针
 * @details 公式: ΔU = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2*e(k-1)+e(k-2))
 *          输出增量累加到当前输出，具有抗积分饱和特性
 */
void incremental_PID(struct I_pid_obj *motor, struct PID_param *pid)
{
    float proportion = 0, integral = 0, differential = 0;

    /* 计算当前偏差 */
    motor->bias = motor->target - motor->measure;

    /* 比例项：偏差变化量 */
    proportion = motor->bias - motor->last_bias;

    /* 积分项：带抗积分饱和处理 */
    if (motor->output > pid->outputMax || motor->measure > pid->actualMax)
    {
        /* 输出饱和上限，仅允许负偏差积分 */
        if (motor->bias < 0)
            integral = motor->bias;
    }
    else if (motor->output < -pid->outputMax || motor->measure < -pid->actualMax)
    {
        /* 输出饱和下限，仅允许正偏差积分 */
        if (motor->bias > 0)
            integral = motor->bias;
    }
    else
    {
        /* 正常积分 */
        integral = motor->bias;
    }

    /* 微分项：二阶差分 */
    differential = (motor->bias - 2 * motor->last_bias + motor->last2_bias);

    /* 增量累加 */
    motor->output += pid->kp * proportion + pid->ki * integral + pid->kd * differential;

    if (motor->output > pid->outputMax)
        motor->output = pid->outputMax;
    else if (motor->output < -pid->outputMax)
        motor->output = -pid->outputMax;

    /* 更新历史偏差 */
    motor->last2_bias = motor->last_bias;
    motor->last_bias = motor->bias;

    if (motor->target == 0.0f && fabsf(motor->measure) < 0.5f)
    {
        motor->output = 0;
        motor->bias = 0;
        motor->last_bias = 0;
        motor->last2_bias = 0;
    }
}

/* ======================== 位置式 PID 算法 ======================== */

/**
 * @brief  位置式 PID 计算（带微分滤波和积分分离）
 * @param  obj PID 对象指针
 * @param  pid PID 参数指针
 * @return float PID 输出值
 * @details 公式: U = Kp*e(k) + Ki*Σe(k) + Kd*Δe(k)
 *          - 积分分离：偏差过小或过零时清零积分
 *          - 微分滤波：一阶低通滤波器平滑微分项
 */
float positional_PID(struct P_pid_obj *obj, struct PID_param *pid)
{
    float differential = 0;

    /* 计算当前偏差 */
    obj->bias = obj->target - obj->measure;

    /* 积分分离条件：偏差很小或过零时清零积分，防止超调 */
    if (fabs(obj->bias) < 0.25f ||
        (obj->bias > 0 && obj->last_bias < 0) ||
        (obj->bias < 0 && obj->last_bias > 0))
    {
        obj->integral = 0;
    }
    else
    {
        /* 抗积分饱和：输出饱和时限制积分方向 */
        if (obj->output >= pid->outputMax)
        {
            /* 输出饱和上限，仅允许负偏差积分 */
            if (obj->bias < 0)
                obj->integral += obj->bias;
        }
        else if (obj->output <= pid->outputMin)
        {
            /* 输出饱和下限，仅允许正偏差积分 */
            if (obj->bias > 0)
                obj->integral += obj->bias;
        }
        else
        {
            /* 正常积分 */
            obj->integral += obj->bias;
        }
    }

    /* 微分项：一阶低通滤波 */
    differential = (obj->bias - obj->last_bias) * pid->differential_filterK +
                   (1 - pid->differential_filterK) * obj->last_differential;

    /* 位置式 PID 输出 */
    obj->output = pid->kp * obj->bias +
                  pid->ki * obj->integral +
                  pid->kd * differential;

    /* 更新历史值 */
    obj->last_bias = obj->bias;
    obj->last_differential = differential;

    return obj->output;
}

/* ======================== PID 参数初始化 ======================== */

/**
 * @brief  PID 参数初始化
 * @details 初始化所有通道的 PID 参数，包括：
 *          - 四个电机的速度 PID
 *          - 循线 PID
 *          - 转弯 PID
 *          - 平滑陀螺仪 PID
 *          - 漂移补偿 PID
 *          - 灰度循线 PID
 */
void pid_init(void)
{
    /* ---------- 左前电机 L0 ---------- */
    motor_pid_paramL0.kp = 10;
    motor_pid_paramL0.ki = 12;
    motor_pid_paramL0.kd = 0.8f;
    motor_pid_paramL0.differential_filterK = 0.5f;
    motor_pid_paramL0.outputMax = MOTOR_PWM_MAX;
    motor_pid_paramL0.actualMax = 500;

    /* ---------- 左后电机 L1 ---------- */
    motor_pid_paramL1.kp = 8.5f;
    motor_pid_paramL1.ki = 4.5f;
    motor_pid_paramL1.kd = 0.5f;
    motor_pid_paramL1.differential_filterK = 0.5f;
    motor_pid_paramL1.outputMax = MOTOR_PWM_MAX;
    motor_pid_paramL1.actualMax = 500;

    /* ---------- 右前电机 R0 ---------- */
    motor_pid_paramR0.kp = 10;
    motor_pid_paramR0.ki = 6;
    motor_pid_paramR0.kd = 0.8f;
    motor_pid_paramR0.differential_filterK = 0.5f;
    motor_pid_paramR0.outputMax = MOTOR_PWM_MAX;
    motor_pid_paramR0.actualMax = 500;

    /* ---------- 右后电机 R1 ---------- */
    motor_pid_paramR1.kp = 10;
    motor_pid_paramR1.ki = 6;
    motor_pid_paramR1.kd = 0.8f;
    motor_pid_paramR1.differential_filterK = 0.5f;
    motor_pid_paramR1.outputMax = MOTOR_PWM_MAX;
    motor_pid_paramR1.actualMax = 500;

    /* ---------- 默认循线 ---------- */
    line_pid_param.kp = 8;
    line_pid_param.ki = 0;
    line_pid_param.kd = 90;
    line_pid_param.differential_filterK = 0.5f;
    line_pid_param.outputMax = 300;
    line_pid_param.outputMin = -300;

    /* ---------- 转弯控制 ---------- */
    gyroT_pid_param.kp = 1.50f;
    gyroT_pid_param.ki = 0;
    gyroT_pid_param.kd = 1.8f;
    gyroT_pid_param.differential_filterK = 1.0f;
    gyroT_pid_param.outputMax = 500;
    gyroT_pid_param.outputMin = -500;

    /* ---------- 平滑陀螺仪 ---------- */
    gyroG_pid_param.kp = 0.7f;
    gyroG_pid_param.ki = 0;
    gyroG_pid_param.kd = 4;
    gyroG_pid_param.differential_filterK = 0.5f;
    gyroG_pid_param.outputMax = 500;
    gyroG_pid_param.outputMin = -500;

    /* ---------- 漂移补偿 ---------- */
    GyroP_pid_param.kp = 0.9f;
    GyroP_pid_param.ki = 0.004f;
    GyroP_pid_param.kd = 0.5f;
    GyroP_pid_param.differential_filterK = 0.5f;
    GyroP_pid_param.outputMax = 100;
    GyroP_pid_param.outputMin = -100;

    /* ---------- 灰度循线 ---------- */
    lineG_pid_param.kp = 15;
    lineG_pid_param.ki = 0;
    lineG_pid_param.kd = 5;
    lineG_pid_param.differential_filterK = 0.5f;
    lineG_pid_param.outputMax = 500;
    lineG_pid_param.outputMin = -500;

    /* 清零电机 PID 状态 */
    motor_pid_clear();
}

/* ======================== PID 参数渐变 ======================== */

/**
 * @brief  PID 参数渐变（线性插值）
 * @param  current 当前参数
 * @param  target  目标参数
 * @param  step    插值步长 (0.0~1.0)，1.0表示立即切换
 * @details 将 current 向 target 方向移动 step 比例
 *          step=0.1 表示每次移动10%，10次控制周期完成过渡
 */
void pid_param_blend(struct PID_param *current, const struct PID_param *target, float step)
{
    if (step >= 1.0f)
    {
        /* 立即切换 */
        *current = *target;
        return;
    }

    current->kp += (target->kp - current->kp) * step;
    current->ki += (target->ki - current->ki) * step;
    current->kd += (target->kd - current->kd) * step;
    current->differential_filterK += (target->differential_filterK - current->differential_filterK) * step;
    current->outputMax += (target->outputMax - current->outputMax) * step;
    current->outputMin += (target->outputMin - current->outputMin) * step;
    current->actualMax += (target->actualMax - current->actualMax) * step;
}

/* ======================== PID 状态管理 ======================== */

/**
 * @brief  清零电机 PID 状态
 * @details 重置四个电机的偏差和输出，用于模式切换或紧急停止
 */
void motor_pid_clear(void)
{
    motor_L0 = (struct I_pid_obj){0, 0, 0, 0, 0, 0};
    motor_L1 = (struct I_pid_obj){0, 0, 0, 0, 0, 0};
    motor_R0 = (struct I_pid_obj){0, 0, 0, 0, 0, 0};
    motor_R1 = (struct I_pid_obj){0, 0, 0, 0, 0, 0};
}

/* ======================== 调试接口函数 ======================== */

/**
 * @brief  USMART 调试接口
 * @param  val  参数值
 * @param  deno 分母
 * @param  mode 模式
 * @note   预留接口，当前未实现
 */
void usmart_pid(uint16_t val, int deno, int mode)
{
    /* 预留调试接口 */
}

/**
 * @brief  修改目标值（调试用）
 * @param  targetq 目标值
 */
void chage_target(uint16_t targetq)
{
    motor_L0.target = sin_generator(&sin1);
}

/**
 * @brief  修改电机 L1 的 Kp 参数（调试用）
 * @param  param 参数值（实际值 = param / 10.0）
 */
void speed_pid_kp(int param)
{
    motor_pid_paramL1.kp = param / 10.0f;
    motor_pid_clear();
}

/**
 * @brief  修改电机 L1 的 Kd 参数（调试用）
 * @param  param 参数值（实际值 = param / 10.0）
 */
void speed_pid_kd(int param)
{
    motor_pid_paramL1.kd = param / 10.0f;
    motor_pid_clear();
}

/**
 * @brief  修改电机 L1 的 Ki 参数（调试用）
 * @param  param 参数值（实际值 = param / 100.0）
 */
void speed_pid_ki(int param)
{
    motor_pid_paramL1.ki = param / 100.0f;
    motor_pid_clear();
}
