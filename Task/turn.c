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
#include "math.h"

#define TURN_DONE_DEG          2.0f
#define TURN_STAGE_DONE_DEG    1.0f
#define TURN_STAGE_TRAVEL_DONE_DEG 135.0f
#define TURN_STAGE_R_RATIO     1.2f
#define TURN_MIN_SPEED         5.0f

/* 角度目标（AngleT=转弯，AngleG=陀螺仪直行） */
struct Angle_Control angle = {0, 0};
volatile uint8_t StageTurn_Flag = 0;
static uint8_t stage_turn_active = 0;
static float stage_turn_last_yaw = 0.0f;
static float stage_turn_travel = 0.0f;

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

static float norm_target(float target)
{
    while (target > 180.0f)   target -= 360.0f;
    while (target <= -180.0f) target += 360.0f;
    return target;
}

static float turn_deadzone_comp(float speed, float err, float done_deg)
{
    if (fabsf(speed) >= TURN_MIN_SPEED || fabsf(err) <= done_deg)
        return speed;

    if (speed > 0.0f)
        return speed + TURN_MIN_SPEED;

    if (speed < 0.0f)
        return speed - TURN_MIN_SPEED;

    if (err < 0.0f)
        return TURN_MIN_SPEED;

    return -TURN_MIN_SPEED;
}

static void turn_done_stop(void)
{
    motor_all.Lspeed = motor_all.Rspeed = 0;
    gyroT_pid.integral = 0;
    gyroT_pid.output = 0;
}

static void stage_turn_reset(void)
{
    stage_turn_active = 0;
    stage_turn_last_yaw = 0.0f;
    stage_turn_travel = 0.0f;
}

static float turn_prepare_measure(float target, uint8_t force_right)
{
    float now = getAngleZ();

    target = norm_target(target);
    gyroT_pid.measure = need2turn(now, target);

    /*
     * 平台 180 度在 +/-180 附近容易选到相反方向。
     * 显式强制右转后，调头方向固定，避免在平台边缘来回横摆。
     */
    if (force_right && gyroT_pid.measure > 0.0f)
        gyroT_pid.measure -= 360.0f;

    gyroT_pid.target = 0;
    return gyroT_pid.measure;
}

static void turn_apply_speed(float right_ratio, float done_deg)
{
    float gt = positional_PID(&gyroT_pid, &gyroT_pid_param);

    /* 低速区按输出方向补足转矩，避免平台摩擦让一侧轮子停住。 */
    gt = turn_deadzone_comp(gt, gyroT_pid.measure, done_deg);
    gt = clampf(gt, motor_all.GyroT_speedMax);

    motor_all.Lspeed =  gt;
    motor_all.Rspeed = -gt * right_ratio;
}

static uint8_t turn_angle_base(float target, float right_ratio,
                               uint8_t force_right, float done_deg)
{
    float err = turn_prepare_measure(target, force_right);

    if (fabsf(err) < done_deg)
    {
        turn_done_stop();
        return 1;
    }

    turn_apply_speed(right_ratio, done_deg);
    return 0;
}

/**
 * @brief  陀螺仪原地转到目标角度
 * @return 1=到位, 0=进行中
 */
uint8_t Turn_Angle(float target)
{
    return turn_angle_base(target, 1.0f, 0, TURN_DONE_DEG);
}

uint8_t Stage_turn_Angle(float target)
{
    float now = getAngleZ();

    /*
     * P1 的循迹板接触地面，角度误差符号会在强摩擦下提前触发完成。
     * 平台 180 改用实际累计 yaw 转角判停，避免只转几十度就退出。
     */
    if (!stage_turn_active)
    {
        stage_turn_active = 1;
        stage_turn_last_yaw = now;
        stage_turn_travel = 0.0f;
    }
    else
    {
        float delta = need2turn(stage_turn_last_yaw, now);
        stage_turn_last_yaw = now;
        stage_turn_travel += fabsf(delta);
    }

    if (stage_turn_travel >= TURN_STAGE_TRAVEL_DONE_DEG)
    {
        stage_turn_reset();
        turn_done_stop();
        return 1;
    }

    turn_prepare_measure(target, 1);
    turn_apply_speed(TURN_STAGE_R_RATIO, TURN_STAGE_DONE_DEG);
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
