/**
 * @file    barrier.c
 * @brief   障碍物处理模块
 * @details 包含zhunbei()准备函数、平台、桥、楼梯等障碍物处理
 */

#include "barrier.h"
#include "../map/map.h"
#include "../chassis/chassis_api.h"
#include "motor_task.h"
#include "encoder.h"
#include "pid.h"
#include "imu.h"
#include "scaner.h"
#include "bsp_linefollower.h"
#include "delay.h"
#include "math.h"
#include "FreeRTOS.h"
#include "task.h"

/* ======================== 控制周期 ======================== */

#define CONTROL_CYCLE_MS        5       /* 控制周期 5ms */

/* ======================== 坡道角度阈值 ======================== */

#define BEGIN_UP     (basic_p + 5.0f)   /* 开始上坡 */
#define UP_PITCH     (basic_p + 20.0f)  /* 上坡中 */
#define AFTER_UP     (basic_p + 5.0f)   /* 上坡结束 */
#define BEGIN_DOWN   (basic_p - 5.0f)   /* 开始下坡 */
#define DOWN_PITCH   (basic_p - 20.0f)  /* 下坡中 */
#define AFTER_DOWN   (basic_p - 5.0f)   /* 下坡结束 */

/* ======================== 速度定义 ======================== */

#define GOSTAGE_SPEED           12      /* 上台速度 */
#define UPDOWN_SPEED_LOW        12      /* 坡道低速 */
#define UPDOWN_SPEED_HIGH       25      /* 坡道高速 */
#define HILL_APPROACH_SPEED     15      /* 楼梯接近速度 */

/* ======================== 延时常量 ======================== */

#define DELAY_STABLE            200     /* 稳定等待 */
#define DELAY_SHORT             100     /* 短暂等待 */

/* ======================== 距离常量 ======================== */

#define DISTANCE_PLATFORM       20      /* 平台前进距离(cm) */
#define DISTANCE_PLATFORM_FRONT 7       /* 平台转身前前进距离(cm) */
#define DISTANCE_PLATFORM_BACK  5       /* 平台转身前后退距离(cm) */
#define DISTANCE_P2_PLATFORM    75      /* P2平台前进距离(cm) */
#define DISTANCE_BRIDGE_ASCEND  15      /* 上桥后稳定距离(cm) */
#define DISTANCE_BRIDGE_TOTAL   65      /* 桥总长度(cm) */
#define DISTANCE_WAVE_ENTRY_MAX 40

/* ======================== 角度常量 ======================== */

#define ANGLE_TURN_180          180.0f  /* 180度转身 */
#define P2_DOWN_BIAS            2.0f
#define BRIDGE_RIGHT_BIAS       0.0f
#define BRIDGE_RED_ANGLE        1.f
#define BRIDGE_RED_LEFT_MASK    0xF800u
#define BRIDGE_RED_RIGHT_MASK   0x007Fu
#define BRIDGE_RED_HOLD_TICKS   30
#define SCANER_CENTER_MASK      0x0180u  /* 中间两路循迹灯 */
#define NODE_ARRIVED_FLAG       0x04u
#define LEFT_LINE_MODE          1
#define RIGHT_LINE_MODE         2
#define CENTER_LINE_MODE        3
#define INVALID_ANGLE           (-1.0f)

/* ======================== 检测阈值 ======================== */

#define RAMP_DETECT_STAGE       20.0f   /* 平台坡道检测阈值(度) */
#define RAMP_DETECT_BRIDGE      5.0f   /* 桥坡道检测阈值(度) */
#define RAMP_DETECT_HILL        15.0f   /* 楼梯坡道检测阈值(度) */
#define GYRO_STABLE_SAMPLES     50      /* 陀螺仪稳定采样次数 */
#define P1_STAGE_APPROACH_SPEED SPEED0
#define P1_STAGE_RAMP_DETECT    10.0f
#define P1_STAGE_LINE_MODE      3

static void line_mode_reset(uint8_t mode)
{
    scaner_set.CatchsensorNum = 0;
    scaner_set.EdgeIgnore = 0;
    LEFT_RIGHT_LINE = mode;
}

static void line_mode_reset_by_flag(u32 flag)
{
    if ((flag & LEFT_LINE) == LEFT_LINE)
        line_mode_reset(LEFT_LINE_MODE);
    else if ((flag & RIGHT_LINE) == RIGHT_LINE)
        line_mode_reset(RIGHT_LINE_MODE);
    else if ((flag & LiuShui) == LiuShui)
        line_mode_reset(CENTER_LINE_MODE);
    else
        line_mode_reset(0);
}

static NODE stage_exit_node(void)
{
    u8 addr;
    u8 exit_num = nodesr.nextNode.nodenum;

    if (exit_num != ROUTE_END)
    {
        addr = getNextConnectNode(nodesr.nowNode.nodenum, exit_num);
        if (Node[addr].nodenum == exit_num)
            return Node[addr];
    }

    if (route[map.point] != ROUTE_END)
    {
        exit_num = route[map.point];
        addr = getNextConnectNode(nodesr.nowNode.nodenum, exit_num);
        if (Node[addr].nodenum == exit_num)
            return Node[addr];
    }

    return nodesr.nowNode;
}

static void barrier_done(uint8_t stop_line, uint8_t clear_pid)
{
    Chassis_ClearMileage();
    if (stop_line)
        motor_all.Cspeed = 0;
    if (clear_pid)
        motor_pid_clear();
    nodesr.nowNode.function = 0;
    nodesr.flag |= NODE_ARRIVED_FLAG;
}

static float bridge_norm_angle(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle <= -180.0f)
        angle += 360.0f;
    return angle;
}

static uint8_t bridge_red_correct(float base_angle, float *tar_angle)
{
    static uint8_t hold = 0;
    static float hold_angle = 0.0f;

    getline_error();

    if (Scaner.detail & BRIDGE_RED_LEFT_MASK)
    {
        hold = BRIDGE_RED_HOLD_TICKS;
        hold_angle = bridge_norm_angle(getAngleZ() + BRIDGE_RED_ANGLE);
        *tar_angle = hold_angle;
        angle.AngleG = *tar_angle;
        motor_all.Gspeed = SPEED1;
        return 1;
    }

    if (Scaner.detail & BRIDGE_RED_RIGHT_MASK)
    {
        hold = BRIDGE_RED_HOLD_TICKS;
        hold_angle = bridge_norm_angle(getAngleZ() - BRIDGE_RED_ANGLE);
        *tar_angle = hold_angle;
        angle.AngleG = *tar_angle;
        motor_all.Gspeed = SPEED1;
        return 1;
    }

    if (hold > 0)
    {
        hold--;
        *tar_angle = hold_angle;
        angle.AngleG = *tar_angle;
        motor_all.Gspeed = SPEED1;
        return 1;
    }

    *tar_angle = base_angle;
    angle.AngleG = *tar_angle;
    motor_all.Gspeed = SPEED2;
    return 0;
}

static void stage_line_ramp_ctrl(RampDir_t dir, float init_speed,
                                 float thresh1, float speed1,
                                 float thresh2, float speed2,
                                 float done_thresh)
{
    enum { RAMP_INIT, RAMP_PHASE1, RAMP_PHASE2 } state = RAMP_INIT;

    /* 阻塞式坡道流程只能在任务上下文调用，内部依赖 vTaskDelay 让出 CPU。 */
    Chassis_MotorControl(is_Line, init_speed, init_speed, 0);
    Chassis_SetTargetSpeed(init_speed);

    while (1)
    {
        float pitch = imu.pitch;

        if (dir == RAMP_ASCEND)
        {
            switch (state)
            {
            case RAMP_INIT:
                if (pitch >= thresh1)
                {
                    Chassis_SetTargetSpeed(speed1);
                    state = RAMP_PHASE1;
                }
                break;
            case RAMP_PHASE1:
                if (pitch >= thresh2)
                {
                    Chassis_SetTargetSpeed(speed2);
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
                    Chassis_SetTargetSpeed(speed1);
                    state = RAMP_PHASE1;
                }
                break;
            case RAMP_PHASE1:
                if (pitch <= thresh2)
                {
                    Chassis_SetTargetSpeed(speed2);
                    state = RAMP_PHASE2;
                }
                break;
            case RAMP_PHASE2:
                if (pitch >= done_thresh) return;
                break;
            }
        }
        vTaskDelay(CONTROL_CYCLE_MS);
    }
}

/* ======================== zhunbei() 准备函数 ======================== */

/**
 * @brief  准备函数 - 启动流程
 * @details 执行顺序：
 *          1. 停车 + 开启红外
 *          2. 等待挡板检测（Infrared_ahead 0->1->0）
 *          3. 陀螺仪离开平台
 *          4. 检测到下坡后切居中巡线
 */
void zhunbei(void)
{
    /* 停车 */
    Chassis_SetMode(is_No);
    motor_all.Lspeed = 0;
    motor_all.Rspeed = 0;

    /* 开启红外 */
    infrare_open = 1;
    vTaskDelay(DELAY_SHORT);

    /* 等待挡板检测 - 碰到挡板 */
    while (Infrared_ahead == 0)
        vTaskDelay(5);

    /* 等待移除挡板 */
    while (Infrared_ahead == 1)
        vTaskDelay(5);

    /* 陀螺仪离开平台 */
    mpuZreset(imu.yaw, nodesr.nowNode.angle);
    angle.AngleG = bridge_norm_angle(getAngleZ() + P2_DOWN_BIAS);
    motor_all.Gincrement = 0.5f;
    motor_all.Gspeed = GOSTAGE_SPEED;
    Chassis_SetMode(is_Gyro);

    /* 检测到下坡 */
    while (imu.pitch > BEGIN_DOWN)
        vTaskDelay(CONTROL_CYCLE_MS);

    /* 切换居中巡线 */
    encoder_clear();
    line_mode_reset(CENTER_LINE_MODE);
    motor_all.Cincrement = 0.5f;
    Chassis_SetTargetSpeed(SPEED1);
    Chassis_SetMode(is_Line);

    /* 等待下坡结束 */
    while (imu.pitch < AFTER_DOWN)
        vTaskDelay(CONTROL_CYCLE_MS);

    /* 清理巡线状态收尾 */
    encoder_clear();
    line_mode_reset(CENTER_LINE_MODE);
    motor_all.Cincrement = 0.5f;
    Chassis_SetTargetSpeed(SPEED1);
}

/* ======================== 通用平台处理（P1/P3/P4等） ======================== */

/**
 * @brief  通用平台处理函数
 * @details 执行顺序：
 *          1. 循线接近，检测坡道（20度）
 *          2. 上坡：init=25, pitch>=basic_p+5→12, pitch>=basic_p+20→12, pitch<=basic_p+5→done
 *          3. 前进20cm到平台
 *          4. 校准航向 + 前进5cm + 后退5cm
 *          5. 刹车 + 180度转身
 *          6. 下坡：init=12, pitch<=basic_p-5→12, pitch<=basic_p-20→25, pitch>=basic_p-5→done
 *          7. 设置到达标志
 */
void Stage(void)
{
    enum {
        STAGE_ASCEND,
        STAGE_TOP,
        STAGE_TURN,
        STAGE_DESCEND,
        STAGE_DONE
    } state = STAGE_ASCEND;

    float origin_angle = 0.0f;
    float approach_speed = SPEED1;
    float ramp_detect = RAMP_DETECT_STAGE;

    if (nodesr.nowNode.nodenum == P1)
    {
        approach_speed = P1_STAGE_APPROACH_SPEED;
        ramp_detect = P1_STAGE_RAMP_DETECT;
        line_mode_reset(P1_STAGE_LINE_MODE);
    }

    /* 循线前进 */
    Chassis_MotorControl(is_Line, approach_speed, approach_speed, 0);
    Chassis_ClearMileage();

    while (state != STAGE_DONE)
    {
        switch (state)
        {
        case STAGE_ASCEND:
            /* 检测坡道（20度） */
            GyroStableReset(GYRO_STABLE_SAMPLES, &origin_angle);

            if (Stage_DetectedRamp(ramp_detect))
            {
                if (origin_angle == 0)
                    origin_angle = getAngleZ();
                if (nodesr.nowNode.nodenum == P1)
                {
                    stage_line_ramp_ctrl(RAMP_ASCEND, UPDOWN_SPEED_HIGH,
                                         BEGIN_UP, UPDOWN_SPEED_LOW,
                                         UP_PITCH, UPDOWN_SPEED_LOW,
                                         AFTER_UP);
                    origin_angle = getAngleZ();
                }
                else
                {
                    RampCtrl_Blocking(RAMP_ASCEND, UPDOWN_SPEED_HIGH, origin_angle,
                                      BEGIN_UP, UPDOWN_SPEED_LOW,
                                      UP_PITCH, UPDOWN_SPEED_LOW,
                                      AFTER_UP, 0);
                }
                state = STAGE_TOP;
            }
            break;

        case STAGE_TOP:
            Chassis_MotorControl(is_Gyro, GOSTAGE_SPEED, GOSTAGE_SPEED, origin_angle);
            while (Infrared_ahead == 1)
                vTaskDelay(CONTROL_CYCLE_MS);
            while (Infrared_ahead == 0)
                vTaskDelay(CONTROL_CYCLE_MS);
            CarBrake();
            vTaskDelay(DELAY_SHORT);

            /* 校准平台航向后前进再后退，给原地转身留空间 */
            mpuZreset(imu.yaw, nodesr.nowNode.angle);
            origin_angle = getAngleZ();
            Chassis_DriveDistance_Blocking(is_Gyro, DISTANCE_PLATFORM_FRONT, GOSTAGE_SPEED, origin_angle);
            CarBrake();
            vTaskDelay(DELAY_SHORT);
            Chassis_DriveDistance_Blocking(is_Gyro, DISTANCE_PLATFORM_BACK, -GOSTAGE_SPEED, origin_angle);
            CarBrake();
            vTaskDelay(DELAY_STABLE);
            state = STAGE_TURN;
            break;

        case STAGE_TURN:
            /* 180度转身 */
            CarBrake();
            vTaskDelay(DELAY_SHORT);
            Chassis_Turn_180_Blocking();
            vTaskDelay(DELAY_SHORT);
            state = STAGE_DESCEND;
            break;

        case STAGE_DESCEND:
        {
            NODE exit_node;

            /* 下坡：init=12, pitch<=basic_p-5→12, pitch<=basic_p-20→25, pitch>=basic_p-5→done */
            RampCtrl_Blocking(RAMP_DESCEND, UPDOWN_SPEED_LOW, getAngleZ(),
                              BEGIN_DOWN, UPDOWN_SPEED_LOW,
                              DOWN_PITCH, UPDOWN_SPEED_HIGH,
                              AFTER_DOWN, 0);

            /* 切换回循线 */
            exit_node = stage_exit_node();
            line_mode_reset_by_flag(exit_node.flag);
            pid_mode_switch_no_inherit(is_No);
            Chassis_SetTargetSpeed(exit_node.speed);
            Chassis_SetMode(is_Line);
            state = STAGE_DONE;
            break;
        }

        default:
            state = STAGE_DONE;
            break;
        }
        vTaskDelay(CONTROL_CYCLE_MS);
    }

    barrier_done(0, 0);
}

/* ======================== P2平台处理 ======================== */

/**
 * @brief  P2平台处理函数
 * @details 执行顺序：
 *          1. 循线找角度
 *          2. 上坡：init=25, pitch>=basic_p+5→12, pitch>=basic_p+20→12, pitch<=basic_p+5→done
 *          3. 前进75cm到平台
 *          4. 刹车 + 180度转身
 *          5. 设置到达标志
 */
void Stage_P2(void)
{
    /* 保存原始PID参数 */
    struct PID_param origin_line = line_pid_param;
    struct PID_param origin_gyro = gyroG_pid_param;

    /* 调整PID参数用于上坡 */
    line_pid_param.kp = 35;
    line_pid_param.ki = 0.004f;
    line_pid_param.kd = 300;

    /* 循线前进，寻找合适的上坡角度 */
    Chassis_MotorControl(is_Line, UPDOWN_SPEED_LOW, UPDOWN_SPEED_LOW, 0);
    Chassis_ClearMileage();

    float tempAngle = INVALID_ANGLE;

    while (Scaner.ledNum < 8)
    {
        getline_error();
        if ((Scaner.detail & SCANER_CENTER_MASK) == SCANER_CENTER_MASK && Scaner.ledNum < 5)
            tempAngle = getAngleZ();
        vTaskDelay(CONTROL_CYCLE_MS);
    }

    if (tempAngle == INVALID_ANGLE)
        tempAngle = getAngleZ();

    /* 上坡：init=25, pitch>=basic_p+5→12, pitch>=basic_p+20→12, pitch<=basic_p+5→done */
    RampCtrl_Blocking(RAMP_ASCEND, UPDOWN_SPEED_HIGH, tempAngle,
                      BEGIN_UP, UPDOWN_SPEED_LOW,
                      UP_PITCH, UPDOWN_SPEED_LOW,
                      AFTER_UP, 0);

    /* 到平台上，前进75cm */
    Chassis_DriveDistance_Blocking(is_Gyro, DISTANCE_P2_PLATFORM, GOSTAGE_SPEED, tempAngle);

    /* 刹车 */
    CarBrake();
    vTaskDelay(DELAY_STABLE);

    /* 180度转身 */
    Chassis_Turn_By_StopGyro_Blocking(getAngleZ() + ANGLE_TURN_180, getAngleZ());

    /* 恢复PID参数 */
    line_pid_param = origin_line;
    gyroG_pid_param = origin_gyro;

    barrier_done(1, 1);
}

/* ======================== 过桥处理 ======================== */

/**
 * @brief  过桥处理函数
 * @details 执行顺序：
 *          1. 循线接近桥，检测坡道
 *          2. 上桥：init=25, pitch>=basic_p+5→25, pitch>=basic_p+20→12, pitch<=basic_p+25→done
 *          3. 继续上坡：init=12, pitch>=basic_p+5→12, pitch<=basic_p+5→done
 *          4. 使用坡道前稳定角锁定桥上航向
 *          5. 桥上直行（陀螺仪锁定）
 *          6. 下桥：init=12, pitch<=basic_p-5→12, pitch<=basic_p-20→20, pitch>=basic_p-5→done
 *          7. 循线收尾
 */
void Barrier_Bridge(void)
{
    enum {
        BRIDGE_APPROACH,    /* 接近：循线检测坡道 */
        BRIDGE_ASCEND,      /* 上桥 */
        BRIDGE_CORRECT,     /* 锁定桥上航向 */
        BRIDGE_ACCELERATE,  /* 桥上直行 */
        BRIDGE_DESCEND,     /* 下桥 */
        BRIDGE_DONE
    } state = BRIDGE_APPROACH;

    float origin_angle = 0.0f;
    float entry_angle = 0.0f;
    float base_angle = 0.0f;
    float tar_angle = 0.0f;

    LEFT_RIGHT_LINE = CENTER_LINE_MODE;
    Chassis_MotorControl(is_Line, SPEED0, SPEED0, 0);
    Chassis_ClearMileage();

    while (state != BRIDGE_DONE)
    {
        switch (state)
        {
        case BRIDGE_APPROACH:
            Chassis_SetMode(is_Line);
            Chassis_SetTargetSpeed(SPEED0);
            GyroStableReset(GYRO_STABLE_SAMPLES, &origin_angle);

            if (Stage_DetectedRamp(RAMP_DETECT_BRIDGE))
            {
                mpuZreset(imu.yaw, nodesr.nowNode.angle);
                origin_angle = nodesr.nowNode.angle;
                entry_angle = bridge_norm_angle(origin_angle + BRIDGE_RIGHT_BIAS);
                Chassis_MotorControl(is_Gyro, SPEED0, SPEED0, entry_angle);
                state = BRIDGE_ASCEND;
            }
            break;

        case BRIDGE_ASCEND:
            /* 上桥：init=25, pitch>=basic_p+5→25, pitch>=basic_p+20→12, pitch<=basic_p+25→done */
            RampCtrl_Blocking(RAMP_ASCEND, UPDOWN_SPEED_HIGH, entry_angle,
                              BEGIN_UP, UPDOWN_SPEED_HIGH,
                              UP_PITCH, UPDOWN_SPEED_LOW,
                              UP_PITCH + 20.0f, 0);

            /* 上桥后：前进15cm稳定 */
            Chassis_ClearMileage();
            Chassis_DriveDistance_Blocking(is_Gyro, DISTANCE_BRIDGE_ASCEND, UPDOWN_SPEED_LOW, entry_angle);

            RampCtrl_Blocking(RAMP_ASCEND, UPDOWN_SPEED_LOW, entry_angle,
                              0, UPDOWN_SPEED_LOW,
                              0, UPDOWN_SPEED_LOW,
                              AFTER_UP, 0);

            Chassis_ClearMileage();
            state = BRIDGE_CORRECT;
            break;

        case BRIDGE_CORRECT:
            base_angle = bridge_norm_angle(origin_angle + BRIDGE_RIGHT_BIAS);
            tar_angle = base_angle;
            Chassis_MotorControl(is_Gyro, SPEED2, SPEED2, tar_angle);
            Chassis_ClearMileage();
            state = BRIDGE_ACCELERATE;
            break;

        case BRIDGE_ACCELERATE:
            /* 桥上直行：陀螺仪锁定 */
            bridge_red_correct(base_angle, &tar_angle);

            if (fabsf(Chassis_GetMileage()) >= DISTANCE_BRIDGE_TOTAL)
            {
                Chassis_MotorControl(is_Gyro, UPDOWN_SPEED_LOW, UPDOWN_SPEED_LOW, tar_angle);
                state = BRIDGE_DESCEND;
            }
            break;

        case BRIDGE_DESCEND:
            /* 下桥：init=12, pitch<=basic_p-5→12, pitch<=basic_p-20→20, pitch>=basic_p-5→done */
            RampCtrl_Blocking(RAMP_DESCEND, UPDOWN_SPEED_LOW, tar_angle,
                              BEGIN_DOWN, UPDOWN_SPEED_LOW,
                              DOWN_PITCH, SPEED0,
                              AFTER_DOWN, 0);

            /* 切换回循线 */
            Chassis_MotorControl(is_Line, SPEED1, SPEED1, 0);

            barrier_done(0, 0);
            state = BRIDGE_DONE;
            break;

        default:
            state = BRIDGE_DONE;
            break;
        }
        vTaskDelay(CONTROL_CYCLE_MS);
    }
}

void Barrier_WavedPlate(float length)
{
    struct PID_param old_line = line_pid_param;
    struct PID_param old_gyro = gyroG_pid_param;
    int8_t old_ignore = scaner_set.EdgeIgnore;
    uint8_t old_mode = LEFT_RIGHT_LINE;

    Chassis_DisableAntiSnake();
    LEFT_RIGHT_LINE = CENTER_LINE_MODE;
    scaner_set.EdgeIgnore = 0;
    Chassis_MotorControl(is_Line, SPEED0, SPEED0, 0);
    Chassis_ClearMileage();

    while (Scaner.ledNum <= 4 || Scaner.lineNum == 1)
    {
        getline_error();
        Cross_getline();
        if ((Cross_Scaner.detail & SCANER_CENTER_MASK) == SCANER_CENTER_MASK)
            mpuZreset(imu.yaw, nodesr.nowNode.angle);
        if (fabsf(Chassis_GetMileage()) >= DISTANCE_WAVE_ENTRY_MAX)
            break;
        vTaskDelay(CONTROL_CYCLE_MS);
    }

    line_pid_param.kp = 35.0f;
    line_pid_param.ki = 0;
    line_pid_param.kd = 0;
    scaner_set.EdgeIgnore = 3;
    Chassis_ClearMileage();
    Chassis_MotorControl(is_Line, UPDOWN_SPEED_LOW, UPDOWN_SPEED_LOW, 0);

    while (fabsf(Chassis_GetMileage()) < length)
        vTaskDelay(CONTROL_CYCLE_MS);

    WavePlateLeft_Flag = 0;
    WavePlateRight_Flag = 0;
    scaner_set.EdgeIgnore = old_ignore;
    LEFT_RIGHT_LINE = old_mode;
    line_pid_param = old_line;
    gyroG_pid_param = old_gyro;
    barrier_done(0, 0);
}

/* ======================== 楼梯处理 ======================== */

/**
 * @brief  楼梯/山地处理函数
 * @details 执行顺序：
 *          1. 循线接近，检测坡道（40度）
 *          2. 上坡：init=12, pitch>=basic_p+5→12, pitch>=basic_p+15→12, pitch<=basic_p+5→done
 *          3. 下坡：init=12, pitch<=basic_p→12, pitch<=basic_p-8→12, pitch>=basic_p-3→done
 *          4. 刹车 + 设置到达标志
 */
void Barrier_Hill(void)
{
    enum {
        HILL_APPROACH,
        HILL_ASCEND,
        HILL_DESCEND,
        HILL_DONE
    } state = HILL_APPROACH;

    float origin_angle = 0.0f;

    Chassis_MotorControl(is_Line, 12, 12, 0);
    vTaskDelay(10);
    Chassis_ClearMileage();

    while (state != HILL_DONE)
    {
        switch (state)
        {
        case HILL_APPROACH:
            GyroStableReset(GYRO_STABLE_SAMPLES, &origin_angle);

            if (Stage_DetectedRamp(RAMP_DETECT_HILL))
            {
                if (origin_angle == 0)
                    origin_angle = getAngleZ();
                Chassis_MotorControl(is_Gyro, HILL_APPROACH_SPEED, HILL_APPROACH_SPEED, origin_angle);
                state = HILL_ASCEND;
            }
            break;

        case HILL_ASCEND:
            /* 上坡：init=12, pitch>=basic_p+5→12, pitch>=basic_p+15→12, pitch<=basic_p+5→done */
            RampCtrl_Blocking(RAMP_ASCEND, UPDOWN_SPEED_LOW, origin_angle,
                              basic_p + 5.0f, UPDOWN_SPEED_LOW,
                              basic_p + 15.0f, UPDOWN_SPEED_LOW,
                              basic_p + 5.0f, 0.05f);
            state = HILL_DESCEND;
            break;

        case HILL_DESCEND:
            /* 下坡：init=12, pitch<=basic_p→12, pitch<=basic_p-8→12, pitch>=basic_p-3→done */
            RampCtrl_Blocking(RAMP_DESCEND, UPDOWN_SPEED_LOW, origin_angle,
                              basic_p, UPDOWN_SPEED_LOW,
                              basic_p - 8.0f, UPDOWN_SPEED_LOW,
                              basic_p - 3.0f, 0.05f);
            state = HILL_DONE;
            break;

        default:
            state = HILL_DONE;
            break;
        }
        vTaskDelay(CONTROL_CYCLE_MS);
    }

    /* 刹车 */
    CarBrake();

    barrier_done(0, 0);
}
