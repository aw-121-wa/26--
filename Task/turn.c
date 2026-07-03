/**
 * @file    turn.c
 * @brief   转弯 / 陀螺仪角度控制
 * @details getAngleZ() 返回带软件补偿的 Z 轴角度（配合 mpuZreset 做基准归零）；
 *          原地转弯与陀螺仪直行均基于位置式 PID（gyroT_pid / gyroG_pid）。
 *          各函数只计算并写入 motor_all.Lspeed/Rspeed，实际下发由 motor_task 完成。
 */

#include "turn.h"
#include "motor_task.h"
#include "imu.h"
#include "pid.h"
#include "map.h"
#include "math.h"

#define P1_STAGE_LEFT_SCALE     0.78f
#define P1_STAGE_RIGHT_SCALE    0.95f
#define P6_STAGE_RIGHT_SCALE    1.1f

/* 角度目标（AngleT=转弯，AngleG=陀螺仪直行） */
struct Angle_Control angle = {0, 0};

/* Turn360 内部状态 */
static float   Turn360RecallAngle = 0;
static uint8_t MustBeZero = 0;

/* ======================== 角度工具 ======================== */

/* 角度差归一化到 (-180, 180] */
static float need2turn(float now_angle, float target_angle)
{
    float diff = target_angle - now_angle;
    while (diff > 180.0f)   diff -= 360.0f;
    while (diff <= -180.0f) diff += 360.0f;
    return diff;
}

/* 限幅到 [-lim, lim] */
static float clampf(float v, float lim)
{
    if (v >  lim) return  lim;
    if (v < -lim) return -lim;
    return v;
}

/* 角度转 0~360 制 */
static float Change360Angle(float a)
{
    if (a < 0) a = 360.0f - fabsf(a);
    return a;
}

/**
 * @brief  当前 Z 轴角度（含软件补偿）
 */
float getAngleZ(void)
{
    float a = imu.yaw + imu.compensateZ;
    while (a > 180.0f)   a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
}

/* ======================== 转弯 ======================== */

/**
 * @brief  陀螺仪原地转到目标角度
 * @return 1=到位, 0=进行中
 */
uint8_t Turn_Angle(float target)
{
    float now = getAngleZ();

    if (fabsf(need2turn(now, target)) < 2.0f)
    {
        motor_all.Lspeed = motor_all.Rspeed = 0;
        gyroT_pid.integral = 0;
        gyroT_pid.output = 0;
        return 1;
    }

    gyroT_pid.measure = need2turn(now, target);
    gyroT_pid.target  = 0;

    float gt = clampf(positional_PID(&gyroT_pid, &gyroT_pid_param), motor_all.GyroT_speedMax);
    motor_all.Lspeed =  gt;
    motor_all.Rspeed = -gt;
    return 0;
}

/**
 * @brief  平台转弯（P1/P6 处左右轮略不对称）
 * @return 1=到位, 0=进行中
 */
uint8_t Stage_turn_Angle(float target)
{
    float now = getAngleZ();

    if (fabsf(need2turn(now, target)) < 2.0f)
    {
        motor_all.Lspeed = motor_all.Rspeed = 0;
        gyroT_pid.integral = 0;
        gyroT_pid.output = 0;
        return 1;
    }

    gyroT_pid.measure = need2turn(now, target);
    gyroT_pid.target  = 0;

    float gt = clampf(positional_PID(&gyroT_pid, &gyroT_pid_param), motor_all.GyroT_speedMax);
    if (nodesr.nowNode.nodenum == P1)
    {
        motor_all.Lspeed = gt * P1_STAGE_LEFT_SCALE;
        motor_all.Rspeed = -gt * P1_STAGE_RIGHT_SCALE;
    }
    else if (nodesr.nowNode.nodenum == P6)
    {
        motor_all.Lspeed = gt;
        motor_all.Rspeed = -gt * P6_STAGE_RIGHT_SCALE;
    }
    else
    {
        motor_all.Lspeed = gt;
        motor_all.Rspeed = -gt;
    }
    return 0;
}

/* ======================== 陀螺仪直行 ======================== */

/**
 * @brief  锁定目标角度直线行驶
 * @param  target_angle 目标航向
 * @param  speed        前进速度
 */
void runWithAngle(float target_angle, float speed)
{
    float now = getAngleZ();

    gyroG_pid.measure = need2turn(now, target_angle);
    gyroG_pid.target  = 0;

    float gg = positional_PID(&gyroG_pid, &gyroG_pid_param) * speed / 20.0f;
    float gg_limit = fabsf(speed) * 0.6f;
    if (gg_limit < motor_all.GyroG_speedMax)
        gg = clampf(gg, gg_limit);
    else
        gg = clampf(gg, motor_all.GyroG_speedMax);

    motor_all.Lspeed = speed + gg;
    motor_all.Rspeed = speed - gg;
}

/* ======================== 360° 自转 ======================== */

/**
 * @brief  单步 360° 自转（需先置 Turn360_Flag 并记录 Turn360RecallAngle）
 */
void Turn360Step(void)
{
    float now;
    if (MustBeZero)
        now = 0;
    else
        now = Change360Angle(Change360Angle(imu.yaw) - Turn360RecallAngle);

    gyroT_pid.measure = 360.0f - now;
    gyroT_pid.target  = 1;

    if (gyroT_pid.measure < 2.0f)
    {
        motor_all.Lspeed = motor_all.Rspeed = 0;
        gyroT_pid.integral = 0;
        gyroT_pid.output = 0;
        Turn360_Flag = 0;
        return;
    }

    float gt = clampf(positional_PID(&gyroT_pid, &gyroT_pid_param), motor_all.GyroT_speedMax);
    motor_all.Lspeed =  gt;
    motor_all.Rspeed = -gt;
}

/* ======================== 速度渐变 ======================== */

/**
 * @brief  速度渐变：Now 朝 target 以 up/down 步进逼近
 */
void gradual_cal(struct Gradual *grad, float target, float up, float down)
{
    grad->Target = target;

    if (grad->Now < grad->Target)
    {
        grad->Now += up;
        if (grad->Now > grad->Target)
            grad->Now = grad->Target;
    }
    else if (grad->Now > grad->Target)
    {
        grad->Now -= down;
        if (grad->Now < grad->Target)
            grad->Now = grad->Target;
    }
}
