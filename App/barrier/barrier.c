/**
 * @file    barrier.c
 * @brief   障碍物处理模块
 * @details 包含zhunbei()准备函数、平台、桥、楼梯等障碍物处理
 */

#include "barrier.h"
#include "../map/map.h"
#include "main_task.h"
#include "../chassis/chassis_api.h"
#include "motor_task.h"
#include "encoder.h"
#include "pid.h"
#include "imu.h"
#include "scaner.h"
#include "bsp_linefollower.h"
#include "delay.h"
#include "lsc16_action.h"
#include "math.h"
#include "stdio.h"
#include "string.h"
#include "usart.h"

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
#define DOOR_WAIT_MS            3000u   /* D点停车等待 */
#define DOOR_STOP_LED_NUM       8u
#define DOOR_APPROACH_TIMEOUT_MS 5000u

/* ======================== 距离常量 ======================== */

#define DISTANCE_PLATFORM       20      /* 平台前进距离(cm) */
#define DISTANCE_PLATFORM_FRONT 5       /* 平台转身前前进距离(cm) */
#define DISTANCE_PLATFORM_BACK  6       /* 平台转身前后退距离(cm) */
#define DISTANCE_P2_PLATFORM    20      /* P2平台前进距离(cm) */
#define DISTANCE_BRIDGE_ASCEND  15      /* 上桥后稳定距离(cm) */
#define DISTANCE_BRIDGE_TOTAL   65      /* 桥总长度(cm) */
#define DISTANCE_WAVE_ENTRY_MAX 40

/* 南极/珠峰参数（优先沿用主参考工程，单位已适配为 cm、cm/s）。 */
#define BARRIER_OLD_SPEED          6.0f
#define BARRIER_DESCEND_SPEED      13.0f
#define BARRIER_LOW_SPEED          20.0f
#define BARRIER_MOUNT_SPEED        22.0f
#define BARRIER_IMPACT_SPEED       16.0f
#define BARRIER_TURN_SPEED_MAX     25.0f
#define BARRIER_SHORT_TIMEOUT_MS   5000u
#define BARRIER_LONG_TIMEOUT_MS    20000u
#define BARRIER_INFRARED_TIMEOUT_MS 5000u
#define BARRIER_IMPACT_MAX_DISTANCE 150.0f
#define BARRIER_CENTER_MASK        0x0180u

/* ======================== 角度常量 ======================== */

#define ANGLE_TURN_180          180.0f  /* 180度转身 */
#define P2_DOWN_BIAS            0.0f
#define BRIDGE_RIGHT_BIAS       1.0f   /* 1.0°左修，抵消机械右偏（上桥用） */
#define BRIDGE_RED_ANGLE        1.0f   /* 桥中左偏需强推 */
#define BRIDGE_RED_LEFT_MASK    0xF800u  /* 传感器11~15，5个 */
#define BRIDGE_RED_RIGHT_MASK   0x001Fu  /* 传感器0~4，5个 */
#define BRIDGE_RED_HOLD_TICKS   20      /* 100ms，缩短响应间隔 */
#define SCANER_CENTER_MASK      0x0180u  /* 中间两路循迹灯 */
#define NODE_ARRIVED_FLAG       0x04u
#define LEFT_LINE_MODE          1
#define RIGHT_LINE_MODE         2
#define CENTER_LINE_MODE        3
#define INVALID_ANGLE           (-1.0f)

typedef struct {
    struct PID_param line_pid;
    struct PID_param gyro_pid;
    float turn_speed_max;
    float liushui_rate;
    int8_t edge_ignore;
    uint8_t line_mode;
} BarrierMotionSnapshot;

/* 珠峰当前所处阶段，供串口停车诊断打印使用：
 * 1=第一段爬坡 2=第二段爬坡 3=过渡平台(红外/前进/后退) 4=180调头 5=下坡 */
volatile uint8_t g_barrier_step = 0;

/* ======================== 检测阈值 ======================== */

#define RAMP_DETECT_STAGE       20.0f   /* 平台坡道检测阈值(度) */
#define RAMP_DETECT_BRIDGE      5.0f    /* 桥坡道检测阈值(度) */
#define RAMP_DETECT_HILL        15.0f   /* 楼梯坡道检测阈值(度) */
#define GYRO_STABLE_SAMPLES     50      /* 陀螺仪稳定采样次数 */
#define P1_STAGE_APPROACH_SPEED SPEED0
#define P1_STAGE_RAMP_DETECT    10.0f
#define P1_STAGE_LINE_MODE      3
#define P3_STAGE_APPROACH_SPEED SPEED4
#define P3_STAGE_RAMP_DETECT    10.0f

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

static void barrier_continue_after_wave(void)
{
    Chassis_ClearMileage();
    nodesr.nowNode.function = 0;
    nodesr.flag &= (uint8_t)(~NODE_ARRIVED_FLAG);
}

static void barrier_motion_save(BarrierMotionSnapshot *snapshot)
{
    snapshot->line_pid = line_pid_param;
    snapshot->gyro_pid = gyroG_pid_param;
    snapshot->turn_speed_max = motor_all.GyroT_speedMax;
    snapshot->liushui_rate = LiuShuiRate;
    snapshot->edge_ignore = scaner_set.EdgeIgnore;
    snapshot->line_mode = LEFT_RIGHT_LINE;
}

static void barrier_motion_restore(const BarrierMotionSnapshot *snapshot)
{
    line_pid_param = snapshot->line_pid;
    gyroG_pid_param = snapshot->gyro_pid;
    motor_all.GyroT_speedMax = snapshot->turn_speed_max;
    LiuShuiRate = snapshot->liushui_rate;
    scaner_set.EdgeIgnore = snapshot->edge_ignore;
    Line_SetTrackModeBumpless(snapshot->line_mode);
}

static uint8_t barrier_wait_expired(TickType_t start, uint32_t timeout_ms)
{
    return ((xTaskGetTickCount() - start) >= pdMS_TO_TICKS(timeout_ms)) ? 1u : 0u;
}

static uint8_t barrier_wait_door_stop_line(void)
{
    TickType_t start = xTaskGetTickCount();

    getline_error();
    while (Scaner.ledNum < DOOR_STOP_LED_NUM)
    {
        getline_error();
        if (Chassis_IsStopLocked() ||
            barrier_wait_expired(start, DOOR_APPROACH_TIMEOUT_MS))
            return 0u;
        vTaskDelay(CONTROL_CYCLE_MS);
    }

    return 1u;
}

static void barrier_door_fail(void)
{
    nodesr.flag &= (uint8_t)(~NODE_ARRIVED_FLAG);
    Chassis_ForceStop(CHASSIS_STOP_BARRIER_FAILED);
}

void Barrier_Door(void)
{
    nodesr.flag &= (uint8_t)(~NODE_ARRIVED_FLAG);
    Chassis_SetTargetSpeed(nodesr.nowNode.speed);
    Chassis_SetMode(is_Line);

    if (!barrier_wait_door_stop_line())
    {
        barrier_door_fail();
        return;
    }

    CarBrake();
    vTaskDelay(pdMS_TO_TICKS(DOOR_WAIT_MS));
    if (Chassis_IsStopLocked())
    {
        barrier_door_fail();
        return;
    }

    nodesr.nowNode.function = NONE;
    nodesr.flag |= NODE_ARRIVED_FLAG;
}

static uint8_t barrier_distance_exceeded(float max_distance)
{
    return (max_distance > 0.0f && fabsf(Chassis_GetMileage()) >= max_distance) ? 1u : 0u;
}

static uint8_t barrier_drive_distance(uint8_t mode, float distance, float speed,
                                      float heading, uint32_t timeout_ms)
{
    TickType_t start = xTaskGetTickCount();

    Chassis_ClearMileage();
    Chassis_MotorControl(mode, speed, speed, heading);

    while (fabsf(Chassis_GetMileage()) < distance)
    {
        if (Chassis_IsStopLocked() || barrier_wait_expired(start, timeout_ms))
            return 0u;
        vTaskDelay(CONTROL_CYCLE_MS);
    }
    return 1u;
}

static uint8_t barrier_reverse_distance(float distance, float speed, float heading)
{
    return barrier_drive_distance(is_Gyro, distance, -fabsf(speed), heading,
                                  BARRIER_SHORT_TIMEOUT_MS);
}

static uint8_t barrier_wait_front_infrared(uint32_t timeout_ms)
{
    TickType_t start = xTaskGetTickCount();

    Chassis_ClearMileage();

    while (Infrared_ahead == 0)
    {
        if (Chassis_IsStopLocked() ||
            barrier_distance_exceeded(BARRIER_IMPACT_MAX_DISTANCE) ||
            barrier_wait_expired(start, timeout_ms))
            return 0u;
        vTaskDelay(CONTROL_CYCLE_MS);
    }
    return 1u;
}

static uint8_t barrier_wait_led_at_least(uint8_t minimum, float max_distance,
                                         uint32_t timeout_ms, float *heading)
{
    TickType_t start = xTaskGetTickCount();

    getline_error();
    if (heading != NULL && (Scaner.detail & BARRIER_CENTER_MASK) == BARRIER_CENTER_MASK)
        *heading = getAngleZ();

    while (Scaner.ledNum < minimum)
    {
        getline_error();
        if (heading != NULL && (Scaner.detail & BARRIER_CENTER_MASK) == BARRIER_CENTER_MASK)
            *heading = getAngleZ();
        if (Chassis_IsStopLocked() || barrier_distance_exceeded(max_distance) ||
            barrier_wait_expired(start, timeout_ms))
            return 0u;
        vTaskDelay(CONTROL_CYCLE_MS);
    }
    return 1u;
}

static uint8_t barrier_wait_led_below(uint8_t maximum, float max_distance,
                                      uint32_t timeout_ms)
{
    TickType_t start = xTaskGetTickCount();

    getline_error();
    while (Scaner.ledNum >= maximum)
    {
        getline_error();
        if (Chassis_IsStopLocked() || barrier_distance_exceeded(max_distance) ||
            barrier_wait_expired(start, timeout_ms))
            return 0u;
        vTaskDelay(CONTROL_CYCLE_MS);
    }
    return 1u;
}

static uint8_t barrier_wait_line_transition(float max_distance)
{
    Chassis_ClearMileage();
    if (!barrier_wait_led_at_least(5u, max_distance, BARRIER_SHORT_TIMEOUT_MS, NULL))
        return 0u;

    Chassis_ClearMileage();
    return barrier_wait_led_below(4u, max_distance, BARRIER_SHORT_TIMEOUT_MS);
}

static uint8_t barrier_wait_pitch_above(float threshold, float max_distance,
                                        uint32_t timeout_ms)
{
    TickType_t start = xTaskGetTickCount();

    while (imu.pitch < threshold)
    {
        if (Chassis_IsStopLocked() || barrier_distance_exceeded(max_distance) ||
            barrier_wait_expired(start, timeout_ms))
            return 0u;
        vTaskDelay(CONTROL_CYCLE_MS);
    }
    return 1u;
}

static uint8_t barrier_wait_pitch_below(float threshold, float max_distance,
                                        uint32_t timeout_ms)
{
    TickType_t start = xTaskGetTickCount();

    while (imu.pitch > threshold)
    {
        if (Chassis_IsStopLocked() || barrier_distance_exceeded(max_distance) ||
            barrier_wait_expired(start, timeout_ms))
            return 0u;
        vTaskDelay(CONTROL_CYCLE_MS);
    }
    return 1u;
}

static float barrier_angle_normalize(float value)
{
    while (value > 180.0f)
        value -= 360.0f;
    while (value <= -180.0f)
        value += 360.0f;
    return value;
}

static void barrier_fail(const BarrierMotionSnapshot *snapshot)
{
    barrier_motion_restore(snapshot);
    nodesr.flag &= (uint8_t)(~NODE_ARRIVED_FLAG);
    Chassis_ForceStop(CHASSIS_STOP_BARRIER_FAILED);
}

static void barrier_complete(const BarrierMotionSnapshot *snapshot, float exit_speed)
{
    if (Chassis_IsStopLocked())
    {
        barrier_fail(snapshot);
        return;
    }

    barrier_motion_restore(snapshot);
    Chassis_MotorControl(is_Line, exit_speed, exit_speed, 0.0f);
    barrier_done(0u, 0u);
}

static float bridge_norm_angle(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle <= -180.0f)
        angle += 360.0f;
    return angle;
}

static uint8_t bridge_red_reset = 0;  /* 跨调用复位标志 */

static uint8_t bridge_red_correct(float base_angle, float *tar_angle)
{
    static uint8_t hold = 0;
    static float hold_angle = 0.0f;
    static uint8_t hold_side = 0;
    static float saved_kp = 0.0f;
    static float bridge_base_kp = 0.0f;

    if (bridge_red_reset)
    {
        hold = 0;
        hold_side = 0;
        saved_kp = gyroG_pid_param.kp;
        bridge_base_kp = gyroG_pid_param.kp;
        bridge_red_reset = 0;
    }

    getline_error();

    if (Scaner.detail & BRIDGE_RED_LEFT_MASK)
    {
        if (hold == 0 || hold_side != 1)
        {
            saved_kp = gyroG_pid_param.kp;
            gyroG_pid_param.kp = saved_kp * 1.8f;
        }
        hold = BRIDGE_RED_HOLD_TICKS;
        hold_side = 1;
        hold_angle = bridge_norm_angle(getAngleZ() + BRIDGE_RED_ANGLE);
        *tar_angle = hold_angle;
        angle.AngleG = *tar_angle;
        motor_all.Gspeed = SPEED1;
        return 1;
    }

    if (Scaner.detail & BRIDGE_RED_RIGHT_MASK)
    {
        if (hold == 0 || hold_side != 2)
        {
            saved_kp = gyroG_pid_param.kp;
            gyroG_pid_param.kp = saved_kp * 1.8f;
        }
        hold = BRIDGE_RED_HOLD_TICKS;
        hold_side = 2;
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
        if (hold == 0)
        {
            gyroG_pid_param.kp = saved_kp;
            hold_side = 0;
        }
        return 1;
    }

    gyroG_pid_param.kp = bridge_base_kp * 1.3f;
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
    Lsc16_RunActionGroupBlocking(LSC16_ACTION_BARRIER_DETECTED,
                                 LSC16_ACTION_RUN_ONCE,
                                 LSC16_WAIT_INIT_MS);

    /* 等待移除挡板 */
    while (Infrared_ahead == 1)
        vTaskDelay(5);
    Lsc16_RunActionGroupBlocking(LSC16_ACTION_TURN_DONE,
                                 LSC16_ACTION_RUN_ONCE,
                                 LSC16_WAIT_STAND_MS);

#if LINE_DEBUG_MODE
    /* 测试模式：挡板移开直接巡线 */
    encoder_clear();
    line_mode_reset(CENTER_LINE_MODE);
    motor_all.Cincrement = 0.5f;
    Chassis_SetTargetSpeed(SPEED3);
    Chassis_SetMode(is_Line);
#else
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
    Chassis_SetTargetSpeed(SPEED0);
    Chassis_SetMode(is_Line);

    /* 等待下坡结束 */
    while (imu.pitch < AFTER_DOWN)
        vTaskDelay(CONTROL_CYCLE_MS);

    /* 清理巡线状态收尾 */
    encoder_clear();
    line_mode_reset(CENTER_LINE_MODE);
    motor_all.Cincrement = 0.5f;
    Chassis_SetTargetSpeed(SPEED0);
#endif
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
    else if (nodesr.nowNode.nodenum == P3)
    {
        approach_speed = P3_STAGE_APPROACH_SPEED;
        ramp_detect = P3_STAGE_RAMP_DETECT;
    }

    /* 循线前进 */
    Chassis_MotorControl(is_Line, approach_speed, approach_speed, 0);
    Chassis_ClearMileage();

    /* 记录上坡前航向（仅一次，避免每次检测前阻塞100ms错过窗口） */
    GyroStableReset(GYRO_STABLE_SAMPLES, &origin_angle);

    while (state != STAGE_DONE)
    {
        switch (state)
        {
        case STAGE_ASCEND:
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
            Lsc16_RunActionGroupBlocking(LSC16_ACTION_BARRIER_DETECTED,
                                         LSC16_ACTION_RUN_ONCE,
                                         LSC16_WAIT_PLATFORM_MS);
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
            Lsc16_RunActionGroupBlocking(LSC16_ACTION_TURN_DONE,
                                         LSC16_ACTION_RUN_ONCE,
                                         LSC16_WAIT_STAND_MS);
            vTaskDelay(DELAY_SHORT);
            //Chassis_DriveDistance_Blocking(is_Gyro, 15.0f, GOSTAGE_SPEED, getAngleZ());
            //CarBrake();
            //Chassis_SetMode(is_No);
            //while (1) { vTaskDelay(100); }
            state = STAGE_DESCEND;
            break;

        case STAGE_DESCEND:
        {
            /* After the turn, lock heading and move until descent begins. */
            Chassis_SetMode(is_Gyro);
            motor_all.Gspeed = UPDOWN_SPEED_LOW;
            angle.AngleG = getAngleZ();

            while (imu.pitch > BEGIN_DOWN)
                vTaskDelay(CONTROL_CYCLE_MS);

            /* 居中巡线下坡 */
            encoder_clear();
            line_mode_reset(CENTER_LINE_MODE);
            Chassis_SetTargetSpeed(SPEED0);
            Chassis_SetMode(is_Line);

            /* 等待下坡结束 */
            while (imu.pitch < AFTER_DOWN)
                vTaskDelay(CONTROL_CYCLE_MS);

            /* 坡底恢复提速 */
            encoder_clear();
            line_mode_reset(CENTER_LINE_MODE);
            motor_all.Cincrement = 0.5f;
            Chassis_SetTargetSpeed(SPEED2);
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

    while (Infrared_ahead == 1)
        vTaskDelay(CONTROL_CYCLE_MS);
    while (Infrared_ahead == 0)
        vTaskDelay(CONTROL_CYCLE_MS);
    Lsc16_RunActionGroupBlocking(LSC16_ACTION_BARRIER_DETECTED,
                                 LSC16_ACTION_RUN_ONCE,
                                 LSC16_WAIT_PLATFORM_MS);

    /* 刹车 */
    CarBrake();
    vTaskDelay(DELAY_STABLE);

    /* 180度转身 */
    Chassis_Turn_By_StopGyro_Blocking(getAngleZ() + ANGLE_TURN_180, getAngleZ());
    Lsc16_RunActionGroupBlocking(LSC16_ACTION_TURN_DONE,
                                 LSC16_ACTION_RUN_ONCE,
                                 LSC16_WAIT_STAND_MS);

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

    bridge_red_reset = 1;  /* 复位静态变量 */

    line_mode_reset_by_flag(nodesr.nowNode.flag);  /* 按节点flag巡线 */
    Chassis_MotorControl(is_Line, SPEED0, SPEED0, 0);

    Chassis_ClearMileage();

    while (state != BRIDGE_DONE)
    {
        switch (state)
        {
        case BRIDGE_APPROACH:
            Chassis_SetMode(is_Line);
            Chassis_SetTargetSpeed(SPEED0);


            /* 走够5cm后才启用坡检测，防分岔口误触 */
            if (fabsf(Chassis_GetMileage()) >= 5.0f &&
                Stage_DetectedRamp(RAMP_DETECT_BRIDGE))
            {
                CarBrake();
                vTaskDelay(800);  /* 停800ms调整姿态 */
                mpuZreset(imu.yaw, nodesr.nowNode.angle);
                origin_angle = nodesr.nowNode.angle;
                entry_angle = bridge_norm_angle(origin_angle + BRIDGE_RIGHT_BIAS);
                Chassis_MotorControl(is_Gyro, SPEED0, SPEED0, entry_angle);
                state = BRIDGE_ASCEND;
            }
            break;

        case BRIDGE_ASCEND:
            /* 上桥：循迹板离地，陀螺仪锁航向上坡 */
            RampCtrl_Blocking(RAMP_ASCEND, UPDOWN_SPEED_HIGH, entry_angle,
                              BEGIN_UP, UPDOWN_SPEED_HIGH,
                              UP_PITCH, UPDOWN_SPEED_LOW,
                              UP_PITCH + 20.0f, 0);

            /* 上桥后：陀螺仪前进15cm稳定 */
            Chassis_ClearMileage();
            Chassis_DriveDistance_Blocking(is_Gyro, DISTANCE_BRIDGE_ASCEND, UPDOWN_SPEED_LOW, entry_angle);

            {
                float compensated = entry_angle - imu.pitch * 0.03f;
                RampCtrl_Blocking(RAMP_ASCEND, UPDOWN_SPEED_LOW, compensated,
                                  0, UPDOWN_SPEED_LOW,
                                  0, UPDOWN_SPEED_LOW,
                                  AFTER_UP, 0);
            }

            Chassis_ClearMileage();
            state = BRIDGE_CORRECT;
            break;

        case BRIDGE_CORRECT:
            base_angle = bridge_norm_angle(origin_angle + BRIDGE_RIGHT_BIAS);
            tar_angle = base_angle;
            angle.AngleG = tar_angle;
            motor_all.Gspeed = SPEED1;  /* 给ACCELERATE初始速度 */
            Chassis_ClearMileage();
            state = BRIDGE_ACCELERATE;
            break;

        case BRIDGE_ACCELERATE:
            /* 桥上直行+边沿检测 */
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
            CarBrake();
            vTaskDelay(300);  /* 停300ms稳定姿态 */
            Chassis_MotorControl(is_Line, SPEED1, SPEED1, 0);

            motor_pid_clear();   /* 清电机PID残值 */
            line_pid_obj.integral = 0;
            line_pid_obj.last_bias = 0;
            line_pid_obj.last_differential = 0;  /* 清循线PID残值 */
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
    float heading;

    Lsc16_RunActionGroupBlocking(LSC16_ACTION_BARRIER_DETECTED,
                                 LSC16_ACTION_RUN_ONCE,
                                 LSC16_WAIT_INIT_MS);

    Chassis_DisableAntiSnake();
    Chassis_DisableLineLostProtection();
    scaner_set.EdgeIgnore = 0;
    Line_SetTrackModeBumpless(CENTER_LINE_MODE);
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

    /* 改用陀螺仪锁头直走：波浪板会摇车身，加大kp/kd抵抗摇摆 */
    heading = getAngleZ();
    line_pid_param.kd = 15.0f;
    gyroG_pid_param.kp = 10.0f;
    gyroG_pid_param.ki = 0.0f;
    gyroG_pid_param.kd = 6.5f;
    Chassis_ClearMileage();
    Chassis_MotorControl(is_Gyro, 20.0f, 20.0f, heading);

    while (fabsf(Chassis_GetMileage()) < length)
        vTaskDelay(CONTROL_CYCLE_MS);

    WavePlateLeft_Flag = 0;
    WavePlateRight_Flag = 0;
    Line_SetTrackModeBumpless(CENTER_LINE_MODE);
    scaner_set.EdgeIgnore = old_ignore;
    line_pid_param = old_line;
    gyroG_pid_param = old_gyro;
    Line_SetTrackModeBumpless(old_mode);
    Chassis_EnableAntiSnake();
    Chassis_EnableLineLostProtection();
    Lsc16_RunActionGroupBlocking(LSC16_ACTION_TURN_DONE,
                                 LSC16_ACTION_RUN_ONCE,
                                 LSC16_WAIT_STAND_MS);
    barrier_continue_after_wave();
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
    float approach_spd = (nodesr.nowNode.nodenum == B5) ? 10.0f : 12.0f;
    float saved_kp = gyroG_pid_param.kp;

    if (nodesr.nowNode.nodenum == B5)
        gyroG_pid_param.kp = 1.5f;

    Chassis_MotorControl(is_Line, approach_spd, approach_spd, 0);
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
        {
            float hill_spd = (nodesr.nowNode.nodenum == B5) ? 10.0f : UPDOWN_SPEED_LOW;
            /* 上坡：B5降速到8减少跑偏，其他用12 */
            RampCtrl_Blocking(RAMP_ASCEND, hill_spd, origin_angle,
                              basic_p + 5.0f, hill_spd,
                              basic_p + 15.0f, hill_spd,
                              basic_p + 5.0f, 0.05f);
            state = HILL_DESCEND;
            break;
        }

        case HILL_DESCEND:
        {
            float hill_spd = (nodesr.nowNode.nodenum == B5) ? 10.0f : UPDOWN_SPEED_LOW;
            /* 下坡：B5降速到8减少跑偏 */
            RampCtrl_Blocking(RAMP_DESCEND, hill_spd, origin_angle,
                              basic_p, hill_spd,
                              basic_p - 8.0f, hill_spd,
                              basic_p - 3.0f, 0.05f);
            state = HILL_DONE;
            break;
        }

        default:
            state = HILL_DONE;
            break;
        }
        vTaskDelay(CONTROL_CYCLE_MS);
    }

    /* 刹车 */
    CarBrake();

    gyroG_pid_param.kp = saved_kp;
    barrier_done(0, 0);
}

/* ======================== 南极 / 珠峰 ======================== */

static uint8_t south_pole_capture_heading(float *heading)
{
    TickType_t start = xTaskGetTickCount();

    *heading = nodesr.nowNode.angle;
    Chassis_ClearMileage();
    Chassis_MotorControl(is_Line, BARRIER_LOW_SPEED, BARRIER_LOW_SPEED, 0.0f);

    while (fabsf(Chassis_GetMileage()) < 150.0f)
    {
        getline_error();
        if ((Scaner.detail & BARRIER_CENTER_MASK) == BARRIER_CENTER_MASK)
            *heading = getAngleZ();
        if (Scaner.ledNum >= 4u || Scaner.lineNum >= 2u)
            return 1u;
        if (Chassis_IsStopLocked() ||
            barrier_wait_expired(start, BARRIER_LONG_TIMEOUT_MS))
            return 0u;
        vTaskDelay(CONTROL_CYCLE_MS);
    }

    /* 主参考以接近距离作为兜底；没有中心灯样本时保留节点航向。 */
    return 1u;
}

static uint8_t south_pole_ascend(float *heading)
{
    TickType_t start;

    line_pid_param.kp = 17.0f;
    line_pid_param.ki = 0.0f;
    line_pid_param.kd = 400.0f;

    Chassis_ClearMileage();
    Chassis_MotorControl(is_Gyro, BARRIER_LOW_SPEED, BARRIER_LOW_SPEED, *heading);
    start = xTaskGetTickCount();
    while (fabsf(Chassis_GetMileage()) < 40.0f && imu.pitch <= basic_p + 10.0f)
    {
        if (Chassis_IsStopLocked() ||
            barrier_wait_expired(start, BARRIER_SHORT_TIMEOUT_MS))
            return 0u;
        vTaskDelay(CONTROL_CYCLE_MS);
    }

    Chassis_ClearMileage();
    Chassis_MotorControl(is_Line, BARRIER_MOUNT_SPEED - 7.0f,
                         BARRIER_MOUNT_SPEED - 7.0f, 0.0f);
    start = xTaskGetTickCount();
    while (fabsf(Chassis_GetMileage()) < 180.0f)
    {
        getline_error();
        if ((Scaner.detail & BARRIER_CENTER_MASK) == BARRIER_CENTER_MASK)
            *heading = getAngleZ();
        if (Scaner.ledNum >= 10u && fabsf(Chassis_GetMileage()) > 80.0f)
            return 1u;
        if (Chassis_IsStopLocked() ||
            barrier_wait_expired(start, BARRIER_LONG_TIMEOUT_MS))
            return 0u;
        vTaskDelay(CONTROL_CYCLE_MS);
    }

    return 1u;
}

static uint8_t south_pole_descend(float heading)
{
    Chassis_MotorControl(is_Gyro, BARRIER_DESCEND_SPEED - 5.0f,
                         BARRIER_DESCEND_SPEED - 5.0f, heading);
    if (!barrier_wait_line_transition(100.0f))
    {
        HAL_UART_Transmit(&huart2, (uint8_t *)"SPd:LT fail\r\n", 14, 0xffff);
        return 0u;
    }

    line_pid_param.kp = 35.0f;
    line_pid_param.ki = 0.0f;
    line_pid_param.kd = 1.5f;
    if (!barrier_drive_distance(is_Line, 40.0f, BARRIER_OLD_SPEED, 0.0f,
                                BARRIER_LONG_TIMEOUT_MS))
    {
        HAL_UART_Transmit(&huart2, (uint8_t *)"SPd:D40 fail\r\n", 15, 0xffff);
        return 0u;
    }

    line_pid_param.kp = 24.0f;
    line_pid_param.ki = 0.0f;
    line_pid_param.kd = 200.0f;
    Chassis_ClearMileage();
    Chassis_MotorControl(is_Line, BARRIER_LOW_SPEED, BARRIER_LOW_SPEED, 0.0f);
    if (!barrier_wait_pitch_below(BEGIN_DOWN, 120.0f, BARRIER_LONG_TIMEOUT_MS))
    {
        HAL_UART_Transmit(&huart2, (uint8_t *)"SPd:DOWN fail\r\n", 16, 0xffff);
        return 0u;
    }

    Chassis_ClearMileage();
    if (!barrier_wait_pitch_above(AFTER_DOWN, 150.0f, BARRIER_LONG_TIMEOUT_MS))
    {
        char buf[48];
        int len = snprintf(buf, sizeof(buf), "SPd:FLAT fail d=%.0f p=%.1f\r\n",
                          (double)Chassis_GetMileage(), (double)imu.pitch);
        HAL_UART_Transmit(&huart2, (uint8_t *)buf, len, 0xffff);
        return 0u;
    }
    return 1u;
}

void Barrier_SouthPole(void)
{
    BarrierMotionSnapshot snapshot;
    float heading;
    float turn_target;

    barrier_motion_save(&snapshot);
    motor_all.GyroT_speedMax = BARRIER_TURN_SPEED_MAX;
    gyroG_pid_param.kp = 0.5f;
    gyroG_pid_param.ki = 0.0f;
    gyroG_pid_param.kd = 0.5f;

    if (!south_pole_capture_heading(&heading))
    {
        HAL_UART_Transmit(&huart2, (uint8_t *)"SP:capture fail\r\n", 17, 0xffff);
        barrier_fail(&snapshot);
        return;
    }
    if (!south_pole_ascend(&heading))
    {
        HAL_UART_Transmit(&huart2, (uint8_t *)"SP:ascend fail\r\n", 16, 0xffff);
        barrier_fail(&snapshot);
        return;
    }

    gyroG_pid_param.kp = 3.0f;
    gyroG_pid_param.ki = 0.0f;
    gyroG_pid_param.kd = 0.0f;
    Chassis_MotorControl(is_Gyro, BARRIER_IMPACT_SPEED - 5.0f,
                         BARRIER_IMPACT_SPEED - 5.0f, heading);
    if (!barrier_wait_front_infrared(BARRIER_INFRARED_TIMEOUT_MS))
    {
        HAL_UART_Transmit(&huart2, (uint8_t *)"SP:IR fail\r\n", 13, 0xffff);
        barrier_fail(&snapshot);
        return;
    }

    CarBrake();
    if (!barrier_drive_distance(is_Gyro, 10.0f, BARRIER_IMPACT_SPEED - 5.0f,
                                heading, BARRIER_SHORT_TIMEOUT_MS))
    {
        HAL_UART_Transmit(&huart2, (uint8_t *)"SP:D10 fail\r\n", 14, 0xffff);
        barrier_fail(&snapshot);
        return;
    }
    CarBrake();
    if (!barrier_reverse_distance(5.0f, 12.0f, heading))
    {
        HAL_UART_Transmit(&huart2, (uint8_t *)"SP:REV fail\r\n", 14, 0xffff);
        barrier_fail(&snapshot);
        return;
    }
    CarBrake();
    mpuZreset(imu.yaw, nodesr.nowNode.angle);

    turn_target = barrier_angle_normalize(getAngleZ() + 180.0f);
    Chassis_Turn_180_Blocking();
    if (Chassis_IsStopLocked())
    {
        HAL_UART_Transmit(&huart2, (uint8_t *)"SP:180 stop-lock\r\n", 19, 0xffff);
        barrier_fail(&snapshot);
        return;
    }
    if (fabsf(barrier_angle_normalize(turn_target - getAngleZ())) > 10.0f)
    {
        char buf[32];
        int len = snprintf(buf, sizeof(buf), "SP:180 ang=%.1f\r\n",
                          (double)fabsf(barrier_angle_normalize(turn_target - getAngleZ())));
        HAL_UART_Transmit(&huart2, (uint8_t *)buf, len, 0xffff);
        barrier_fail(&snapshot);
        return;
    }

    HAL_UART_Transmit(&huart2, (uint8_t *)"SP:descend start\r\n", 18, 0xffff);
    if (!south_pole_descend(turn_target))
    {
        HAL_UART_Transmit(&huart2, (uint8_t *)"SP:descend fail\r\n", 17, 0xffff);
        barrier_fail(&snapshot);
        return;
    }

    HAL_UART_Transmit(&huart2, (uint8_t *)"SP:done\r\n", 9, 0xffff);
    barrier_complete(&snapshot, BARRIER_LOW_SPEED);
}

static uint8_t high_mountain_first_ascend(float *heading)
{
    TickType_t start;

    g_barrier_step = 1;
    *heading = nodesr.nowNode.angle;
    line_pid_param.kp = 17.0f;
    line_pid_param.ki = 0.0f;
    line_pid_param.kd = 200.0f;
    scaner_set.EdgeIgnore = 3;
    Chassis_ClearMileage();
    Chassis_MotorControl(is_Line, BARRIER_MOUNT_SPEED - 6.0f,
                         BARRIER_MOUNT_SPEED - 6.0f, 0.0f);

    start = xTaskGetTickCount();
    getline_error();
    while ((Scaner.ledNum > 0u && Scaner.ledNum < 3u) || Scaner.lineNum == 1u)
    {
        getline_error();
        if ((Scaner.detail & BARRIER_CENTER_MASK) == BARRIER_CENTER_MASK)
            *heading = getAngleZ();
        if (barrier_distance_exceeded(120.0f) || Chassis_IsStopLocked() ||
            barrier_wait_expired(start, BARRIER_LONG_TIMEOUT_MS))
            return 0u;
        vTaskDelay(CONTROL_CYCLE_MS);
    }

    Chassis_ClearMileage();
    if (!barrier_wait_pitch_above(BEGIN_UP, 80.0f, BARRIER_LONG_TIMEOUT_MS))
        return 0u;

    Cross_getline();
    Chassis_ClearMileage();
    start = xTaskGetTickCount();
    while (Cross_Scaner.ledNum != 0u)
    {
        getline_error();
        Cross_getline();
        if ((Scaner.detail & BARRIER_CENTER_MASK) == BARRIER_CENTER_MASK)
            *heading = getAngleZ();
        if (barrier_distance_exceeded(100.0f) || Chassis_IsStopLocked() ||
            barrier_wait_expired(start, BARRIER_LONG_TIMEOUT_MS))
            return 0u;
        vTaskDelay(CONTROL_CYCLE_MS);
    }

    Chassis_ClearMileage();
    Chassis_MotorControl(is_Gyro, BARRIER_MOUNT_SPEED - 5.0f,
                         BARRIER_MOUNT_SPEED - 5.0f, *heading);
    if (!barrier_wait_pitch_below(AFTER_UP, 120.0f, BARRIER_LONG_TIMEOUT_MS))
        return 0u;

    return barrier_wait_line_transition(80.0f);
}

static uint8_t high_mountain_second_ascend(float *heading)
{
    g_barrier_step = 2;
    scaner_set.EdgeIgnore = 3;
    if (!barrier_drive_distance(is_Line, 5.0f, BARRIER_MOUNT_SPEED - 5.0f,
                                0.0f, BARRIER_SHORT_TIMEOUT_MS))
        return 0u;

    Chassis_ClearMileage();
    Chassis_MotorControl(is_Line, BARRIER_MOUNT_SPEED - 5.0f,
                         BARRIER_MOUNT_SPEED - 5.0f, 0.0f);
    if (!barrier_wait_led_at_least(10u, 120.0f, BARRIER_LONG_TIMEOUT_MS, heading))
        return 0u;

    scaner_set.EdgeIgnore = 0;
    return 1u;
}

static uint8_t high_mountain_descend(float heading, float normal_liushui_rate)
{
    g_barrier_step = 5;
    Chassis_MotorControl(is_Gyro, BARRIER_OLD_SPEED, BARRIER_OLD_SPEED, heading);
    if (!barrier_wait_line_transition(100.0f))
        return 0u;

    line_pid_param.kp = 35.0f;
    line_pid_param.ki = 0.0f;
    line_pid_param.kd = 1.5f;
    if (!barrier_drive_distance(is_Line, 25.0f, BARRIER_OLD_SPEED, 0.0f,
                                BARRIER_SHORT_TIMEOUT_MS))
        return 0u;

    line_pid_param.kp = 27.0f;
    line_pid_param.ki = 0.0f;
    line_pid_param.kd = 400.0f;
    LiuShuiRate = 2.1f;
    Chassis_ClearMileage();
    Chassis_MotorControl(is_Line, BARRIER_DESCEND_SPEED - 4.0f,
                         BARRIER_DESCEND_SPEED - 4.0f, 0.0f);
    if (!barrier_wait_led_at_least(10u, 140.0f, BARRIER_LONG_TIMEOUT_MS, &heading))
        return 0u;

    LiuShuiRate = normal_liushui_rate;
    Chassis_ClearMileage();
    Chassis_MotorControl(is_Gyro, BARRIER_DESCEND_SPEED,
                         BARRIER_DESCEND_SPEED, heading);
    if (!barrier_wait_pitch_below(BEGIN_DOWN, 120.0f, BARRIER_LONG_TIMEOUT_MS))
        return 0u;
    if (!barrier_wait_pitch_above(AFTER_DOWN, 120.0f, BARRIER_LONG_TIMEOUT_MS))
        return 0u;

    if (!barrier_wait_line_transition(100.0f))
        return 0u;

    line_pid_param.kp = 35.0f;
    line_pid_param.ki = 0.0f;
    line_pid_param.kd = 1.5f;
    if (!barrier_drive_distance(is_Line, 15.0f, BARRIER_OLD_SPEED, 0.0f,
                                BARRIER_SHORT_TIMEOUT_MS))
        return 0u;

    line_pid_param.kp = 24.0f;
    line_pid_param.ki = 0.0f;
    line_pid_param.kd = 400.0f;
    if (!barrier_drive_distance(is_Line, 30.0f, BARRIER_DESCEND_SPEED - 5.0f,
                                0.0f, BARRIER_SHORT_TIMEOUT_MS))
        return 0u;

    Chassis_ClearMileage();
    Chassis_MotorControl(is_Line, BARRIER_DESCEND_SPEED + 4.0f,
                         BARRIER_DESCEND_SPEED + 4.0f, 0.0f);
    if (!barrier_wait_pitch_below(BEGIN_DOWN, 100.0f, BARRIER_LONG_TIMEOUT_MS))
        return 0u;

    Chassis_ClearMileage();
    return barrier_wait_pitch_above(AFTER_DOWN, 120.0f, BARRIER_LONG_TIMEOUT_MS);
}

void Barrier_HighMountain(void)
{
    BarrierMotionSnapshot snapshot;
    float heading;
    float turn_target;

    barrier_motion_save(&snapshot);
    motor_all.GyroT_speedMax = BARRIER_TURN_SPEED_MAX;
    Chassis_DisableLineLostProtection();
    gyroG_pid_param.kp = 0.8f;
    gyroG_pid_param.ki = 0.0f;
    gyroG_pid_param.kd = 4.0f;

    if (!high_mountain_first_ascend(&heading))
    {
        barrier_fail(&snapshot);
        return;
    }
    if (!high_mountain_second_ascend(&heading))
    {
        barrier_fail(&snapshot);
        return;
    }

    g_barrier_step = 3;
    Chassis_MotorControl(is_Gyro, 10.0f, 10.0f, heading);
    if (!barrier_wait_front_infrared(BARRIER_INFRARED_TIMEOUT_MS) ||
        !barrier_drive_distance(is_Gyro, 10.0f, 10.0f, heading,
                                BARRIER_SHORT_TIMEOUT_MS))
    {
        barrier_fail(&snapshot);
        return;
    }

    CarBrake();
    if (!barrier_reverse_distance(6.0f, 12.0f, heading))
    {
        barrier_fail(&snapshot);
        return;
    }
    CarBrake();
    mpuZreset(imu.yaw, nodesr.nowNode.angle);

    turn_target = barrier_angle_normalize(getAngleZ() + 180.0f);
    g_barrier_step = 4;
    Chassis_Turn_180_Blocking();
    if (Chassis_IsStopLocked() ||
        fabsf(barrier_angle_normalize(turn_target - getAngleZ())) > 10.0f)
    {
        barrier_fail(&snapshot);
        return;
    }

    if (!high_mountain_descend(turn_target, snapshot.liushui_rate))
    {
        barrier_fail(&snapshot);
        return;
    }

    Chassis_EnableLineLostProtection();
    barrier_complete(&snapshot, BARRIER_DESCEND_SPEED + 5.0f);
}
