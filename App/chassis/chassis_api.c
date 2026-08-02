/**
 * @file    chassis_api.c
 * @brief   底盘控制 API（中间层）
 * @details 对上层（map/barrier）提供统一的底盘行为接口，对下把命令翻译成
 *          motor_task 真正消费的量：循线速度 motor_all.Cspeed、陀螺仪速度
 *          motor_all.Gspeed、陀螺仪航向 angle.AngleG、转弯目标 angle.AngleT。
 */

#include "chassis_api.h"
#include "motor_task.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "imu.h"
#include "scaner.h"
#include "delay.h"
#include "math.h"
#include "../map/map.h"

/* ======================== 控制周期常量 ======================== */

#define CONTROL_CYCLE_MS        5
#define DELAY_TURN              50
#define RAMP_CTRL_CYCLE_MS      5
#define TURN_STOP_DEADBAND      3.0f
#define TURN_180_DEADBAND       2.0f
#define TURN_180_SPEED          25.0f
#define TURN_180_KP             4.0f
#define TURN_180_KD             70.0f
#define TURN_180_KI             0.0f
#define TIPOVER_ROLL_LIMIT      45.0f
#define TIPOVER_CLEAR_LIMIT     20.0f
#define TIPOVER_CONFIRM_COUNT   3
#define YAW_JUMP_WINDOW_COUNT   20      /* 20 * 5ms = 100ms */
#define YAW_JUMP_LIMIT          360.0f  /* 100ms 内累计 yaw 变化阈值 */

/* ======================== 底盘内部状态 ======================== */

struct Chassis_State {
    uint8_t roll_protect_enabled; /* 侧翻保护使能 */
    uint8_t tipover_count;      /* 侧翻连续确认次数 */
    uint8_t yaw_protect_enabled; /* yaw 突变保护使能 */
    uint8_t yaw_inited;         /* yaw 窗口是否已有首样本 */
    uint8_t yaw_idx;            /* yaw 滑动窗口写入位置 */
    uint8_t yaw_count;          /* yaw 窗口有效样本数 */
    float yaw_last;             /* 上一次原始 imu.yaw */
    float yaw_sum;              /* 窗口内累计绝对变化量 */
    float yaw_delta[YAW_JUMP_WINDOW_COUNT]; /* 100ms yaw 差值环形缓冲 */
    uint8_t stop_locked;        /* 强制停车锁存 */
    Chassis_StopReason_t stop_reason; /* 首个触发原因 */
};

static struct Chassis_State chassis = {0};

/* 角度差归一化到 (-180, 180] */
static float norm180(float diff)
{
    while (diff > 180.0f)   diff -= 360.0f;
    while (diff <= -180.0f) diff += 360.0f;
    return diff;
}

static float roll_offset(void)
{
    return fabsf(imu.roll - basic_r);
}

static void yaw_guard_reset(void)
{
    chassis.yaw_inited = 0;
    chassis.yaw_idx = 0;
    chassis.yaw_count = 0;
    chassis.yaw_last = 0.0f;
    chassis.yaw_sum = 0.0f;

    for (uint8_t i = 0; i < YAW_JUMP_WINDOW_COUNT; i++)
        chassis.yaw_delta[i] = 0.0f;
}

static float yaw_delta(float last, float now)
{
    return fabsf(norm180(now - last));
}

static void stop_lock_set(Chassis_StopReason_t reason)
{
    if (reason == CHASSIS_STOP_NONE)
        return;

    if (!chassis.stop_locked)
        chassis.stop_reason = reason;

    chassis.stop_locked = 1;
}

static void stop_lock_clear(void)
{
    if (chassis.stop_reason == CHASSIS_STOP_TIPOVER &&
        roll_offset() > TIPOVER_CLEAR_LIMIT)
    {
        return;
    }

    chassis.stop_locked = 0;
    chassis.stop_reason = CHASSIS_STOP_NONE;
    chassis.tipover_count = 0;
    yaw_guard_reset();
}

static uint8_t action_timeout(TickType_t start, uint32_t timeout_ms)
{
    return ((xTaskGetTickCount() - start) >= pdMS_TO_TICKS(timeout_ms)) ? 1u : 0u;
}

static void line_pid_by_speed(float speed)
{
    int target = (int)(fabsf(speed) + 0.5f);

    switch (target)
    {
    case SPEED5:
    case SPEED4:
        line_pid_param.kp = 6.0f;
        line_pid_param.ki = 0;
        line_pid_param.kd = 250;
        break;
    case SPEED3:
        line_pid_param.kp = 10.0f;
        line_pid_param.ki = 0;
        line_pid_param.kd = 115;
        break;
    case SPEED25:
        line_pid_param.kp = 12.0f;
        line_pid_param.ki = 0;
        line_pid_param.kd = 140;
        break;
    case SPEED2:
        line_pid_param.kp = 12.0f;
        line_pid_param.ki = 0;
        line_pid_param.kd = 110;
        break;
    case SPEED0:
    case SPEED1:
        line_pid_param.kp = 13.0f;
        line_pid_param.ki = 0;
        line_pid_param.kd = 130;
        break;
    case 12:
    case 15:
        line_pid_param.kp = 20.0f;
        line_pid_param.ki = 0;
        line_pid_param.kd = 60;
        break;
    default:
        break;
    }
}

/* ======================== 坡道控制 ======================== */

/**
 * @brief  坡道阻塞控制（三阶段 pitch 状态机）
 * @details 上坡：pitch>=thresh1→speed1，pitch>=thresh2→speed2，pitch<=done→完成
 *          下坡：pitch<=thresh1→speed1，pitch<=thresh2→speed2，pitch>=done→完成
 * @param  aim 全程锁定的陀螺仪航向
 */
ChassisActionResult_t RampCtrl_Blocking(RampDir_t dir, float init_speed, float aim,
                                        float thresh1, float speed1,
                                        float thresh2, float speed2,
                                        float done_thresh, float GrayCorrectAngle,
                                        uint32_t timeout_ms)
{
    enum { RAMP_INIT, RAMP_PHASE1, RAMP_PHASE2 } state = RAMP_INIT;
    TickType_t start = xTaskGetTickCount();

    (void)GrayCorrectAngle;     /* 预留参数，本工程未用灰度修正 */

    /* 阻塞式坡道流程只能在任务上下文调用，内部依赖 vTaskDelay 让出 CPU。 */
    Chassis_SetMode(is_Gyro);
    if (Chassis_IsStopLocked())
        return CHASSIS_ACTION_STOPPED;
    if (timeout_ms == 0u)
        return CHASSIS_ACTION_SENSOR_FAULT;

    motor_all.Gspeed = init_speed;
    angle.AngleG = aim;

    while (1)
    {
        float pitch = imu.pitch;
        angle.AngleG = aim;     /* 全程锁定航向 */

        if (dir == RAMP_ASCEND)
        {
            switch (state)
            {
            case RAMP_INIT:
                if (pitch >= thresh1)
                {
                    motor_all.Gspeed = speed1;
                    state = RAMP_PHASE1;
                }
                break;
            case RAMP_PHASE1:
                if (pitch >= thresh2)
                {
                    motor_all.Gspeed = speed2;
                    state = RAMP_PHASE2;
                }
                break;
            case RAMP_PHASE2:
                if (pitch <= done_thresh) return CHASSIS_ACTION_OK;
                break;
            }
        }
        else
        {
            switch (state)
            {
            case RAMP_INIT:
                if (pitch <= thresh1)
                {
                    motor_all.Gspeed = speed1;
                    state = RAMP_PHASE1;
                }
                break;
            case RAMP_PHASE1:
                if (pitch <= thresh2)
                {
                    motor_all.Gspeed = speed2;
                    state = RAMP_PHASE2;
                }
                break;
            case RAMP_PHASE2:
                if (pitch >= done_thresh) return CHASSIS_ACTION_OK;
                break;
            }
        }

        if (Chassis_IsStopLocked())
            return CHASSIS_ACTION_STOPPED;

        if (action_timeout(start, timeout_ms))
        {
            Chassis_ForceStop(CHASSIS_STOP_MOTION_TIMEOUT);
            return CHASSIS_ACTION_TIMEOUT;
        }

        vTaskDelay(RAMP_CTRL_CYCLE_MS);
    }
}

/* ======================== 底盘控制 ======================== */

/**
 * @brief  设置工作模式（等价于 pid_mode_switch，提供统一 API）
 */
void Chassis_SetMode(uint8_t mode)
{
    pid_mode_switch(mode);
}

/**
 * @brief  设置工作模式与速度/航向
 * @param  aim 陀螺仪模式下的锁定航向
 */
void Chassis_MotorControl(uint8_t mode, float lspeed, float rspeed, float aim)
{
    Chassis_SetMode(mode);
    if (Chassis_IsStopLocked())
        return;

    motor_all.Lspeed = lspeed;
    motor_all.Rspeed = rspeed;

    if (mode == is_Line)
    {
        motor_all.Cspeed = lspeed;
    }
    else if (mode == is_Gyro)
    {
        motor_all.Gspeed = lspeed;
        angle.AngleG = aim;
    }
}

/**
 * @brief  设置循线目标速度
 */
void Chassis_SetTargetSpeed(float speed)
{
    if (Chassis_IsStopLocked())
        return;

    line_pid_by_speed(speed);

    if (PIDMode == is_Gyro)
        motor_all.Gspeed = speed;
    else
        motor_all.Cspeed = speed;
}

/**
 * @brief  设置陀螺仪直行目标航向
 */
void Chassis_SetGyroAngle_Go(float aim)
{
    angle.AngleG = aim;
}

/**
 * @brief  清零里程
 */
void Chassis_ClearMileage(void)
{
    encoder_clear();
    motor_all.Distance = 0;
}

/**
 * @brief  获取里程
 */
float Chassis_GetMileage(void)
{
    return motor_all.Distance;
}

/**
 * @brief  刹车
 */
void CarBrake(void)
{
    Chassis_SetMode(is_No);
    motor_all.Lspeed = 0;
    motor_all.Rspeed = 0;
}

/**
 * @brief  强制停车锁存。普通刹车仍直接使用 CarBrake()，不设置故障原因。
 */
void Chassis_ForceStop(Chassis_StopReason_t reason)
{
    if (reason == CHASSIS_STOP_NONE)
        return;

    stop_lock_set(reason);
    yaw_guard_reset();
    CarBrake();
}

uint8_t Chassis_IsStopLocked(void)
{
    return chassis.stop_locked;
}

Chassis_StopReason_t Chassis_GetStopReason(void)
{
    return chassis.stop_reason;
}

void Chassis_ClearStopLock(void)
{
    stop_lock_clear();
}

/**
 * @brief  行驶指定距离（阻塞）
 * @param  aim 陀螺仪模式下的锁定航向
 */
ChassisActionResult_t Chassis_DriveDistance_Blocking(uint8_t mode, float distance,
                                                      float speed, float aim,
                                                      uint32_t timeout_ms)
{
    TickType_t start;

    if (distance <= 0.0f || fabsf(speed) < 1.0f || timeout_ms == 0u)
        return CHASSIS_ACTION_SENSOR_FAULT;

    Chassis_ClearMileage();
    Chassis_SetMode(mode);
    if (Chassis_IsStopLocked())
        return CHASSIS_ACTION_STOPPED;
    start = xTaskGetTickCount();

    if (mode == is_Gyro)
    {
        motor_all.Gspeed = speed;
        angle.AngleG = aim;
    }
    else
    {
        motor_all.Cspeed = speed;
    }

    while (fabsf(motor_all.Distance) < distance && !Chassis_IsStopLocked())
    {
        if (action_timeout(start, timeout_ms))
        {
            Chassis_ForceStop(CHASSIS_STOP_MOTION_TIMEOUT);
            return CHASSIS_ACTION_TIMEOUT;
        }
        vTaskDelay(CONTROL_CYCLE_MS);
    }
    return Chassis_IsStopLocked() ? CHASSIS_ACTION_STOPPED : CHASSIS_ACTION_OK;
}

static ChassisActionResult_t chassis_turn_blocking(float target_angle, float deadband,
                                                    uint8_t stage_turn, uint32_t timeout_ms)
{
    TickType_t start = xTaskGetTickCount();
    ChassisActionResult_t result = CHASSIS_ACTION_OK;

    if (timeout_ms == 0u)
        return CHASSIS_ACTION_SENSOR_FAULT;
    StageTurn_Flag = stage_turn;
    Chassis_SetMode(is_Turn);
    if (Chassis_IsStopLocked())
    {
        StageTurn_Flag = 0;
        return CHASSIS_ACTION_STOPPED;
    }

    angle.AngleT = target_angle;

    while (PIDMode == is_Turn && !Chassis_IsStopLocked())
    {
        if (stage_turn && StageTurn_Flag == 0)
            break;
        if (!stage_turn && fabsf(norm180(target_angle - getAngleZ())) <= deadband)
            break;
        if (action_timeout(start, timeout_ms))
        {
            Chassis_ForceStop(CHASSIS_STOP_MOTION_TIMEOUT);
            result = CHASSIS_ACTION_TIMEOUT;
            break;
        }
        vTaskDelay(CONTROL_CYCLE_MS);
    }

    StageTurn_Flag = 0;
    Chassis_SetMode(is_No);
    vTaskDelay(DELAY_TURN);
    if (result == CHASSIS_ACTION_OK && Chassis_IsStopLocked())
        result = CHASSIS_ACTION_STOPPED;
    return result;
}

/**
 * @brief  原地转弯到目标角度（阻塞）
 */
ChassisActionResult_t Chassis_Turn_By_StopGyro_Blocking(float target_angle,
                                                        float current_angle,
                                                        uint32_t timeout_ms)
{
    (void)current_angle;
    return chassis_turn_blocking(target_angle, TURN_STOP_DEADBAND, 0, timeout_ms);
}

ChassisActionResult_t Chassis_Turn_180_Blocking(uint32_t timeout_ms)
{
    struct PID_param old_turn = gyroT_pid_param;
    float old_speed = motor_all.GyroT_speedMax;
    ChassisActionResult_t result;

    motor_all.GyroT_speedMax = TURN_180_SPEED;
    gyroT_pid_param.kp = TURN_180_KP;
    gyroT_pid_param.kd = TURN_180_KD;
    gyroT_pid_param.ki = TURN_180_KI;

    result = chassis_turn_blocking(getAngleZ() + 180.0f, TURN_180_DEADBAND, 1, timeout_ms);
    CarBrake();
    vTaskDelay(300);

    motor_all.GyroT_speedMax = old_speed;
    gyroT_pid_param = old_turn;
    return result;
}

/* ======================== 辅助函数 ======================== */

/**
 * @brief  陀螺仪稳定校准（采样取平均）
 */
void GyroStableReset(uint8_t samples, float *angle_out)
{
    float sum = 0;
    for (uint8_t i = 0; i < samples; i++)
    {
        sum += getAngleZ();
        vTaskDelay(CONTROL_CYCLE_MS);
    }
    *angle_out = sum / samples;
}

/**
 * @brief  检测是否进入坡道（pitch 偏离基准超过阈值）
 */
uint8_t Stage_DetectedRamp(float pitch_thresh)
{
    return (fabsf(imu.pitch - basic_p) > pitch_thresh) ? 1 : 0;
}

/* ======================== 强制停车 / 防护 ======================== */

void Chassis_EnableRollProtection(void)
{
    chassis.roll_protect_enabled = 1;
    chassis.tipover_count = 0;
}

void Chassis_DisableRollProtection(void)
{
    chassis.roll_protect_enabled = 0;
    chassis.tipover_count = 0;
}

void Chassis_EnableYawJumpProtection(void)
{
    chassis.yaw_protect_enabled = 1;
    yaw_guard_reset();
}

void Chassis_DisableYawJumpProtection(void)
{
    chassis.yaw_protect_enabled = 0;
    yaw_guard_reset();
}

uint8_t Chassis_IsTipoverLocked(void)
{
    return (chassis.stop_reason == CHASSIS_STOP_TIPOVER &&
            chassis.stop_locked) ? 1 : 0;
}

void Chassis_ClearTipoverLock(void)
{
    if (chassis.stop_reason == CHASSIS_STOP_TIPOVER)
        Chassis_ClearStopLock();
}

static uint8_t yaw_guard_update(void)
{
    float now;
    float delta;

    if (!chassis.yaw_protect_enabled)
        return 0;

    /*
     * 使用原始 imu.yaw，不使用 getAngleZ()，避免 mpuZreset()
     * 改变软件补偿值时造成误判。
     */
    now = imu.yaw;
    if (!chassis.yaw_inited)
    {
        chassis.yaw_last = now;
        chassis.yaw_inited = 1;
        return 0;
    }

    delta = yaw_delta(chassis.yaw_last, now);
    chassis.yaw_last = now;

    if (chassis.yaw_count < YAW_JUMP_WINDOW_COUNT)
    {
        chassis.yaw_count++;
    }
    else
    {
        chassis.yaw_sum -= chassis.yaw_delta[chassis.yaw_idx];
    }

    chassis.yaw_delta[chassis.yaw_idx] = delta;
    chassis.yaw_sum += delta;
    chassis.yaw_idx++;
    if (chassis.yaw_idx >= YAW_JUMP_WINDOW_COUNT)
        chassis.yaw_idx = 0;

    if (chassis.yaw_sum > YAW_JUMP_LIMIT)
    {
        Chassis_ForceStop(CHASSIS_STOP_YAW_JUMP);
        return 1;
    }

    return 0;
}

static uint8_t roll_guard_update(void)
{
    if (!chassis.roll_protect_enabled)
        return 0;

    if (roll_offset() >= TIPOVER_ROLL_LIMIT)
    {
        chassis.tipover_count++;
        if (chassis.tipover_count >= TIPOVER_CONFIRM_COUNT)
        {
            chassis.tipover_count = 0;
            Chassis_ForceStop(CHASSIS_STOP_TIPOVER);
            return 1;
        }
    }
    else
    {
        chassis.tipover_count = 0;
    }

    return 0;
}

/**
 * @brief  底盘 5ms 周期更新（由 motor_task 调用）
 * @details 周期保护只保留 yaw 突变和侧翻。
 */
void Chassis_Periodic_Update_5ms(void)
{
    if (chassis.stop_locked)
    {
        CarBrake();
        return;
    }

    if (yaw_guard_update())
        return;

    if (roll_guard_update())
        return;
}
