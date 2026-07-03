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
#define TURN_180_DEADBAND       4.0f
#define TURN_180_SPEED          25.0f
#define TURN_180_KP             4.0f
#define TURN_180_KD             70.0f
#define TURN_180_KI             0.0f

/* ======================== 底盘内部状态 ======================== */

struct Chassis_State {
    float target_speed;         /* 当前目标速度备份（供 anti-snake 恢复用） */
    uint8_t anti_snake_flag;    /* 游龙防护激活标志 */
    int16_t anti_snake_count;   /* 游龙偏移计数 */
    float saved_line_kp;        /* anti-snake 前循线 kp 备份 */
    float saved_line_kd;        /* anti-snake 前循线 kd 备份 */
    uint8_t line_lost_enabled;  /* 丢线保护使能 */
    int16_t line_lost_count;    /* 连续丢线计数 */
    uint8_t roll_protect_enabled;
    uint8_t tipover_locked;
    uint8_t tipover_count;
};

static struct Chassis_State chassis = {0};

#define TIPOVER_ROLL_LIMIT      45.0f
#define TIPOVER_CLEAR_LIMIT     20.0f
#define TIPOVER_CONFIRM_COUNT   3

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

static void line_pid_by_speed(float speed)
{
    int target = (int)(fabsf(speed) + 0.5f);

    switch (target)
    {
    case SPEED5:
    case SPEED4:
        line_pid_param.kp = 4.0f;
        line_pid_param.ki = 0;
        line_pid_param.kd = 250;
        break;
    case SPEED3:
        line_pid_param.kp = 7.0f;
        line_pid_param.ki = 0;
        line_pid_param.kd = 115;
        break;
    case SPEED25:
        line_pid_param.kp = 8.0f;
        line_pid_param.ki = 0;
        line_pid_param.kd = 140;
        break;
    case SPEED2:
        line_pid_param.kp = 8.0f;
        line_pid_param.ki = 0;
        line_pid_param.kd = 110;
        break;
    case SPEED0:
    case SPEED1:
        line_pid_param.kp = 8.5f;
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
void RampCtrl_Blocking(RampDir_t dir, float init_speed, float aim,
                       float thresh1, float speed1,
                       float thresh2, float speed2,
                       float done_thresh, float GrayCorrectAngle)
{
    enum { RAMP_INIT, RAMP_PHASE1, RAMP_PHASE2 } state = RAMP_INIT;

    (void)GrayCorrectAngle;     /* 预留参数，本工程未用灰度修正 */

    /* 阻塞式坡道流程只能在任务上下文调用，内部依赖 vTaskDelay 让出 CPU。 */
    Chassis_SetMode(is_Gyro);
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
                if (pitch <= done_thresh) return;
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
                if (pitch >= done_thresh) return;
                break;
            }
        }
        vTaskDelay(RAMP_CTRL_CYCLE_MS);
    }
}

/* ======================== 底盘控制 ======================== */

static void anti_snake_restore_pid(void)
{
    if (chassis.saved_line_kp <= 0.0f)
        return;

    line_pid_param.kp = chassis.saved_line_kp;
    line_pid_param.kd = chassis.saved_line_kd;
    chassis.saved_line_kp = 0.0f;
    chassis.saved_line_kd = 0.0f;
}

static void line_guard_soft_clear(void)
{
    /*
     * 离开循线时只清保护计数，不清 PID 历史和速度渐变。
     * Line/Gyro 的控制状态继承统一交给 motor_task.c 处理。
     */
    anti_snake_restore_pid();
    chassis.anti_snake_flag = 0;
    chassis.anti_snake_count = 0;
    chassis.line_lost_count = 0;
}

static void chassis_leave_line_soft(uint8_t next_mode)
{
    if (PIDMode == is_Line && next_mode != is_Line)
        line_guard_soft_clear();
}

/**
 * @brief  设置工作模式（等价于 pid_mode_switch，提供统一 API）
 */
void Chassis_SetMode(uint8_t mode)
{
    chassis_leave_line_soft(mode);
    pid_mode_switch(mode);
}

/**
 * @brief  设置工作模式与速度/航向
 * @param  aim 陀螺仪模式下的锁定航向
 */
void Chassis_MotorControl(uint8_t mode, float lspeed, float rspeed, float aim)
{
    Chassis_SetMode(mode);
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
 * @brief  设置循线目标速度（备份供 anti-snake 恢复）
 */
void Chassis_SetTargetSpeed(float speed)
{
    chassis.target_speed = speed;
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
 * @brief  行驶指定距离（阻塞）
 * @param  aim 陀螺仪模式下的锁定航向
 */
void Chassis_DriveDistance_Blocking(uint8_t mode, float distance, float speed, float aim)
{
    if (distance <= 0.0f)
        return;

    Chassis_ClearMileage();
    Chassis_SetMode(mode);

    if (mode == is_Gyro)
    {
        motor_all.Gspeed = speed;
        angle.AngleG = aim;
    }
    else
    {
        motor_all.Cspeed = speed;
    }

    while (fabsf(motor_all.Distance) < distance)
        vTaskDelay(CONTROL_CYCLE_MS);
}

static void chassis_turn_blocking(float target_angle, float deadband, uint8_t stage_turn)
{
    StageTurn_Flag = stage_turn;
    Chassis_SetMode(is_Turn);
    angle.AngleT = target_angle;

    while (PIDMode == is_Turn &&
           fabsf(norm180(target_angle - getAngleZ())) > deadband)
        vTaskDelay(CONTROL_CYCLE_MS);

    Chassis_SetMode(is_No);
    StageTurn_Flag = 0;
    vTaskDelay(DELAY_TURN);
}

/**
 * @brief  原地转弯到目标角度（阻塞）
 */
void Chassis_Turn_By_StopGyro_Blocking(float target_angle, float current_angle)
{
    (void)current_angle;
    chassis_turn_blocking(target_angle, TURN_STOP_DEADBAND, 0);
}

void Chassis_Turn_180_Blocking(void)
{
    struct PID_param old_turn = gyroT_pid_param;
    float old_speed = motor_all.GyroT_speedMax;

    motor_all.GyroT_speedMax = TURN_180_SPEED;
    gyroT_pid_param.kp = TURN_180_KP;
    gyroT_pid_param.kd = TURN_180_KD;
    gyroT_pid_param.ki = TURN_180_KI;

    chassis_turn_blocking(getAngleZ() + 180.0f, TURN_180_DEADBAND, 1);
    CarBrake();
    vTaskDelay(300);

    motor_all.GyroT_speedMax = old_speed;
    gyroT_pid_param = old_turn;
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

/* ======================== 游龙防护 / 丢线保护 ======================== */

/**
 * @brief  使能游龙防护（检测大幅偏移时自动减速 + 强化 PID）
 */
void Chassis_EnableAntiSnake(void)
{
    chassis.anti_snake_flag = 1;
    chassis.anti_snake_count = 0;
}

void Chassis_DisableAntiSnake(void)
{
    anti_snake_restore_pid();
    chassis.anti_snake_flag = 0;
    chassis.anti_snake_count = 0;
}

/**
 * @brief  使能丢线保护
 */
void Chassis_EnableLineLostProtection(void)
{
    chassis.line_lost_enabled = 1;
    chassis.line_lost_count = 0;
}

/**
 * @brief  关闭丢线保护
 */
void Chassis_DisableLineLostProtection(void)
{
    chassis.line_lost_enabled = 0;
    chassis.line_lost_count = 0;
}

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

uint8_t Chassis_IsTipoverLocked(void)
{
    return chassis.tipover_locked;
}

void Chassis_ClearTipoverLock(void)
{
    if (roll_offset() <= TIPOVER_CLEAR_LIMIT)
    {
        chassis.tipover_locked = 0;
        chassis.tipover_count = 0;
    }
}

#define LINE_LOST_THRESHOLD  200    /* 200 * 5ms = 1 秒 */

/**
 * @brief  底盘 5ms 周期更新（由 motor_task 调用）
 * @details 在循线模式下执行：
 *          1. 游龙防护：检测 Scaner.detail 偏移过大 → 减速 + 强化 kp/kd
 *          2. 丢线保护：连续丢线超 1 秒 → 刹车
 *          参考 xunbao 的 Chassis_Periodic_Update_5ms。
 */
void Chassis_Periodic_Update_5ms(void)
{
    if (chassis.tipover_locked)
    {
        CarBrake();
        return;
    }

    if (chassis.roll_protect_enabled)
    {
        if (roll_offset() >= TIPOVER_ROLL_LIMIT)
        {
            chassis.tipover_count++;
            if (chassis.tipover_count >= TIPOVER_CONFIRM_COUNT)
            {
                chassis.tipover_locked = 1;
                chassis.tipover_count = 0;
                CarBrake();
                return;
            }
        }
        else
        {
            chassis.tipover_count = 0;
        }
    }

    if (PIDMode != is_Line)
        return;

    /* ---- 游龙防护 ---- */
    if (chassis.anti_snake_flag)
    {
        if (Scaner.detail & 0xFC3F)         /* 偏移过大（边缘灯亮） */
        {
            chassis.anti_snake_count++;
        }
        else if (chassis.anti_snake_count > 0)  /* 命中过后回正 → 迅速衰减 */
        {
            if (chassis.anti_snake_count < 200)
                chassis.anti_snake_count -= 10;
        }

        /* 警戒解除条件：回正 or 累计过高（防止死锁） */
        if ((chassis.anti_snake_count <= 0 && chassis.saved_line_kp > 0.0f) ||
            chassis.anti_snake_count >= 200)
        {
            anti_snake_restore_pid();
            chassis.anti_snake_flag = 0;
            chassis.anti_snake_count = 0;
            motor_all.Cspeed = chassis.target_speed;    /* 恢复原速 */
        }
    }

    /* 游龙命中：首次命中时备份原始 PID，然后减速 + 强化循线 PID */
    if (chassis.anti_snake_count > 0)
    {
        if (chassis.anti_snake_count == 1)  /* 首次命中 → 备份 */
        {
            chassis.saved_line_kp = line_pid_param.kp;
            chassis.saved_line_kd = line_pid_param.kd;
        }
        motor_all.Cspeed = chassis.target_speed / 2;    /* 减半 */
        line_pid_param.kp = 12.0f;
        line_pid_param.ki = 0;
        line_pid_param.kd = 200.0f;
    }

    /* ---- 丢线保护 ---- */
    if (chassis.line_lost_enabled)
    {
        if (Scaner.ledNum == 0 && Scaner.lineNum == 0)
        {
            chassis.line_lost_count++;
            if (chassis.line_lost_count >= LINE_LOST_THRESHOLD)
            {
                chassis.line_lost_count = 0;
                chassis.line_lost_enabled = 0;      /* 一次性触发 */
                CarBrake();
            }
        }
        else
        {
            chassis.line_lost_count = 0;
        }
    }
}
