/**
 * @file    map.c
 * @brief   地图管理和Cross状态机模块
 * @details 包含地图初始化、Cross节点间处理状态机、到达判断、障碍物分发
 *          所有运动通过 chassis_api 控制。
 */

#include "map.h"
#include "../chassis/chassis_api.h"
#include "../barrier/barrier.h"
#include "scaner.h"
#include "motor_task.h"
#include "imu.h"
#include "delay.h"
#include "math.h"
#include "bsp_linefollower.h"

/* ======================== 控制周期和延时常量 ======================== */

#define CONTROL_CYCLE_MS        5       /* 控制周期 5ms */
#define STOP_TURN_SETTLE_MS     100u    /* 停车转向前机械稳定时间 */
#define N2_B1_PASS_CM           10.0f
#define NODE_ARRIVED_FLAG       0x04u
#define LEFT_LINE_MODE          1
#define RIGHT_LINE_MODE         2
#define CENTER_LINE_MODE        3
#define SCANER_LEFT_BRANCH_MASK 0x0180u  /* 中间偏左两路循迹灯 */
#define SCANER_RIGHT_BRANCH_MASK 0x0018u /* 中间偏右两路循迹灯 */
#define SCANER_CENTER_MASK      0x0180u
#define ROUTE_HALF_RATIO        0.5f
#define ROUTE_DETECT_RATIO      0.3f
#define ROUTE_SLOW_RATIO        0.7f
#define ROUTE_SEARCH_RATIO      1.15f
#define ROUTE_FAULT_RATIO       1.60f
#define ARRIVE_CONFIRM_SAMPLES  3u
#define TEMP_TRACK_CLEAR_CM     10.0f
#define TURN_NEED_ANGLE         10.0f

/* ======================== 保护阈值 ======================== */

#define MAP_TURN_TIMEOUT_MS     1500u   /* 行进转弯硬超时 */
#define MAP_CLEARANCE_TIMEOUT_MS 3000u  /* 节点清出距离硬超时 */
#define MAP_STOP_TURN_TIMEOUT_MS 4000u  /* 停车转向硬超时 */
#define TURN_TIMEOUT_CYCLES     (MAP_TURN_TIMEOUT_MS / CONTROL_CYCLE_MS)
#define TURN_OSCILLATE_NEAR     8.0f    /* 接近目标阈值(度) */
#define TURN_OSCILLATE_FAR      20.0f   /* 震荡回弹阈值(度) */
#define TURN_STOP_ANGLE         90.0f
#define TURN_DONE_DEADBAND      3.0f
#define TURN_RUN_SPEED_MAX      6.0f    /* 行进转弯差速上限 */
#define TURN_RUN_KD_BOOST       15.0f   /* 行进转弯临时kd，抑制震荡 */

/* ======================== 全局变量定义 ======================== */

struct Map_State map = {0, 0};
NODESR nodesr;
uint8_t isAllRoute = 1;

/* 默认路线：P2 -> N2 -> B1 -> N1 -> P1 */
u8 route[100] = {N2, B1, N1, P1, N1, B2, N4, N5, N6, P4, N6, N5, N4, N3, P3, N3, N4, B3, N2, P2, ROUTE_END};

/* ======================== 底层驱动封装 ======================== */

/**
 * @brief  从节点标志解析循线模式
 * @param  flag 节点标志位
 * @return 循线模式
 */
static uint8_t track_mode_from_flag(u32 flag)
{
    if ((flag & LEFT_LINE) == LEFT_LINE)
        return LEFT_LINE_MODE;
    if ((flag & RIGHT_LINE) == RIGHT_LINE)
        return RIGHT_LINE_MODE;
    if ((flag & LiuShui) == LiuShui)
        return CENTER_LINE_MODE;
    return 0u;
}

uint8_t Map_ValidateData(void)
{
    uint8_t node;
    uint16_t index;

    if (Address[0] != 0u || Address[MAP_NODE_COUNT] != MAP_CONNECTION_COUNT)
        return 0u;

    for (node = 0u; node < MAP_NODE_COUNT; node++)
    {
        if (Address[node + 1u] !=
            (uint8_t)(Address[node] + ConnectionNum[node]))
        {
            return 0u;
        }
    }

    for (index = 0u; index < MAP_CONNECTION_COUNT; index++)
    {
        if (Node[index].nodenum >= MAP_NODE_COUNT)
            return 0u;
    }

    return 1u;
}

static uint8_t map_load_next_node(uint8_t from, uint8_t target)
{
    uint8_t index = getNextConnectNode(from, target);

    if (index == MAP_NODE_INDEX_INVALID)
    {
        Chassis_ForceStop(CHASSIS_STOP_ROUTE_INVALID);
        return 0u;
    }

    nodesr.nextNode = Node[index];
    return 1u;
}

/* ======================== 地图初始化 ======================== */

void mapInit(void)
{
    map.routetime = 0;
    map.point = 0;
    nodesr.flag = 0;
    Cross_reset();
    Chassis_EnableRollProtection();
    Chassis_EnableYawJumpProtection();

    /* 起点：P2平台 */
    nodesr.nowNode.nodenum = P2;
    nodesr.nowNode.angle = 0;
    nodesr.nowNode.function = NONE;
    nodesr.nowNode.speed = SPEED1;
    nodesr.nowNode.step = 10;
    nodesr.nowNode.flag = CLEFT | RIGHT_LINE;

    /* 获取第一个目标节点 */
    if (route[map.point] == ROUTE_END ||
        !map_load_next_node(nodesr.nowNode.nodenum, route[map.point]))
    {
        Chassis_ForceStop(CHASSIS_STOP_ROUTE_INVALID);
        return;
    }
    map.point++;
}

void mapInit1(void)
{
    map.point = 0;
    nodesr.flag = 0;
    Cross_reset();

    nodesr.nowNode.function = NONE;
    nodesr.nowNode.speed = SPEED0;
    nodesr.nowNode.step = 0;

    if (route[map.point] == ROUTE_END ||
        !map_load_next_node(nodesr.nowNode.nodenum, route[map.point]))
    {
        Chassis_ForceStop(CHASSIS_STOP_ROUTE_INVALID);
        return;
    }
    map.point++;
}

/* ======================== 节点连接查找 ======================== */

u8 getNextConnectNode(u8 nownode, u8 nextnode)
{
    unsigned char rest;
    unsigned char addr;

    if (nownode >= MAP_NODE_COUNT || nextnode >= MAP_NODE_COUNT)
        return MAP_NODE_INDEX_INVALID;

    rest = ConnectionNum[nownode];
    addr = Address[nownode];

    for (int i = 0; i < rest; i++)
    {
        if (Node[addr].nodenum == nextnode)
            return addr;
        addr++;
    }
    return MAP_NODE_INDEX_INVALID;
}

/* ======================== 转弯角度计算 ======================== */

/**
 * @brief  计算从当前角度到目标角度需要转多少度
 * @param  current 当前角度
 * @param  target  目标角度
 * @return float   需要转动的角度（正=右转，负=左转）
 */
static float need2turn(float current, float target)
{
    float diff = target - current;
    while (diff > 180.0f)  diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

/* ======================== 到达判断 ======================== */

/**
 * @brief  路口到达判断
 * @param  scaner    循迹器数据指针
 * @param  node_flag 节点标志位
 * @return uint8_t   1=到达, 0=未到达
 */
uint8_t deal_arrive(volatile void *scaner, u32 node_flag)
{
    volatile SCANER *s = (volatile SCANER *)scaner;

    /* 左横线检测 */
    if ((node_flag & DLEFT) == DLEFT)
    {
        uint8_t left_count = 0;
        for (uint8_t i = 8; i < 16; i++)
        {
            if (s->detail & (1 << i))
                left_count++;
        }
        if (left_count >= 5)
            return 1;
    }

    /* 右横线检测 */
    if ((node_flag & DRIGHT) == DRIGHT)
    {
        uint8_t right_count = 0;
        for (uint8_t i = 0; i < 8; i++)
        {
            if (s->detail & (1 << i))
                right_count++;
        }
        if (right_count >= 5)
            return 1;
    }

    /* 左岔路检测 */
    if ((node_flag & CLEFT) == CLEFT)
    {
        if ((s->detail & SCANER_LEFT_BRANCH_MASK) == SCANER_LEFT_BRANCH_MASK && s->ledNum < 6)
            return 1;
    }

    /* 右岔路检测 */
    if ((node_flag & CRIGHT) == CRIGHT)
    {
        if ((s->detail & SCANER_RIGHT_BRANCH_MASK) == SCANER_RIGHT_BRANCH_MASK && s->ledNum < 6)
            return 1;
    }

    /* 多条变一条 */
    if ((node_flag & MUL2SING) == MUL2SING)
    {
        if (s->lineNum >= 2 && s->ledNum <= 4)
            return 1;
    }

    /* 多条变多条 */
    if ((node_flag & MUL2MUL) == MUL2MUL)
    {
        if (s->lineNum >= 3)
            return 1;
    }

    /* 全黑/全白 */
    if ((node_flag & AWHITE) == AWHITE)
    {
        if (s->ledNum == 0 || s->ledNum >= 14)
            return 1;
    }

    /* 更多灯亮 */
    if ((node_flag & MORELED) == MORELED)
    {
        if (s->ledNum >= 5)
            return 1;
    }

    return 0;
}

/* ======================== 障碍物分发 ======================== */

MapPostTurnAction_t map_function(u8 fun)
{
    BarrierResult_t result = BARRIER_RESULT_OK;
    MapPostTurnAction_t post_turn = MAP_POST_TURN_NORMAL;

    switch (fun)
    {
        case NONE:
            break;
        case UpStage:
            result = Stage();
            post_turn = MAP_POST_TURN_SKIP;
            break;
        case Bridge:
            result = Barrier_Bridge();
            break;
        case Hill:
            result = Barrier_Hill();
            break;
        case LBHill:
            result = Barrier_DoubleHill();
            break;
        case SM:
            result = Barrier_SwordMountain();
            break;
        case View:
            result = Barrier_View(0u);
            break;
        case View1:
            result = Barrier_View(1u);
            break;
        case BACK:
            result = Barrier_Back();
            post_turn = MAP_POST_TURN_SKIP;
            break;
        case BSoutPole:
            result = Barrier_SouthPole();
            post_turn = MAP_POST_TURN_SKIP;
            break;
        case QQB:
            result = Barrier_Seesaw();
            break;
        case BLBS:
            result = Barrier_WavedPlate(87.0f);
            break;
        case BLBL:
            result = Barrier_WavedPlate(160.0f);
            break;
        case DOOR:
            result = Barrier_Door(0u);
            break;
        case BHM:
            result = Barrier_HighMountain();
            post_turn = MAP_POST_TURN_SKIP;
            break;
        case IGNORE:
            result = Barrier_Ignore();
            break;
        case UNDER:
            result = Barrier_Under();
            break;
        case Special_node:
            result = Barrier_SpecialNode();
            break;
        case DOOR1:
            result = Barrier_Door(1u);
            break;
        case UpStageP2:
            result = Stage_P2();
            post_turn = MAP_POST_TURN_SKIP;
            break;
        default:
            result = BARRIER_RESULT_SENSOR_FAULT;
            break;
    }

    if (result != BARRIER_RESULT_OK)
    {
        if (!Chassis_IsStopLocked())
            Chassis_ForceStop(CHASSIS_STOP_BARRIER_FAILED);
        return MAP_POST_TURN_SKIP;
    }

    return post_turn;
}

/* ======================== Cross 状态机 ======================== */

typedef enum {
    CROSS_ROUTE_INIT = 0,
    CROSS_ROUTE_CONFIGURED,
    CROSS_ROUTE_TRACKING,
    CROSS_ROUTE_P2_RIGHT
} CrossRouteState_t;

/* Cross 状态机内部状态（由 Cross_reset 重置） */
static CrossRouteState_t route_state = CROSS_ROUTE_INIT;
static uint8_t is_near_end = 0;
static uint8_t detect_started = 0;
static uint8_t yaw_reset_applied = 0;

typedef enum {
    ARRIVE_MULTI_WAIT_MULTI = 0,
    ARRIVE_MULTI_WAIT_SINGLE,
    ARRIVE_MULTI_WAIT_MULTI_AGAIN
} ArriveMultiPhase_t;

typedef enum {
    TEMP_TRACK_FINAL = 0,
    TEMP_TRACK_PRIMARY,
    TEMP_TRACK_CLEARANCE
} TempTrackPhase_t;

static struct {
    uint8_t simple_count;
    uint8_t multi_count;
    ArriveMultiPhase_t multi_phase;
} arrival_detector;

static TempTrackPhase_t temp_track_phase = TEMP_TRACK_FINAL;
static float temp_switch_mileage = 0.0f;

uint8_t Cross_GetState(void)
{
    return (uint8_t)route_state;
}

static void cross_node_advance(void);

static uint8_t route_is_p2_to_n2(void)
{
    return (nodesr.nowNode.nodenum == P2 && nodesr.nextNode.nodenum == N2) ? 1 : 0;
}

static uint8_t route_arrived(void)
{
    return ((nodesr.flag & NODE_ARRIVED_FLAG) == NODE_ARRIVED_FLAG) ? 1 : 0;
}

static void route_set_arrived(void)
{
    nodesr.flag |= NODE_ARRIVED_FLAG;
}

static void route_clear_arrived(void)
{
    nodesr.flag &= (u8)(~NODE_ARRIVED_FLAG);
}

static uint8_t route_need_turn(float ad, float ad2)
{
    if (ad < TURN_NEED_ANGLE || ad2 < TURN_NEED_ANGLE)
        return 0;
    if ((nodesr.nowNode.flag & NOTURN) == NOTURN)
        return 0;
    return 1;
}

static void arrival_detector_reset(void)
{
    arrival_detector.simple_count = 0;
    arrival_detector.multi_count = 0;
    arrival_detector.multi_phase = ARRIVE_MULTI_WAIT_MULTI;
}

static uint8_t arrival_multi_phase_confirm(uint8_t condition)
{
    if (condition)
    {
        if (arrival_detector.multi_count < ARRIVE_CONFIRM_SAMPLES)
            arrival_detector.multi_count++;
    }
    else
    {
        arrival_detector.multi_count = 0;
    }

    if (arrival_detector.multi_count < ARRIVE_CONFIRM_SAMPLES)
        return 0;

    arrival_detector.multi_count = 0;
    return 1;
}

static uint8_t arrival_multiline_update(volatile SCANER *s, u32 node_flag)
{
    uint8_t confirmed;

    if ((node_flag & MUL2SING) == MUL2SING)
    {
        if (arrival_detector.multi_phase == ARRIVE_MULTI_WAIT_MULTI)
        {
            confirmed = arrival_multi_phase_confirm(
                (s->lineNum > 1 && s->ledNum >= 4) ? 1u : 0u);
            if (confirmed)
                arrival_detector.multi_phase = ARRIVE_MULTI_WAIT_SINGLE;
            return 0;
        }

        confirmed = arrival_multi_phase_confirm((s->lineNum == 1) ? 1u : 0u);
        if (confirmed)
        {
            arrival_detector.multi_phase = ARRIVE_MULTI_WAIT_MULTI;
            return 1;
        }
        return 0;
    }

    if ((node_flag & MUL2MUL) == MUL2MUL)
    {
        switch (arrival_detector.multi_phase)
        {
        case ARRIVE_MULTI_WAIT_MULTI:
            confirmed = arrival_multi_phase_confirm(
                (s->lineNum > 1 && s->ledNum >= 4) ? 1u : 0u);
            if (confirmed)
                arrival_detector.multi_phase = ARRIVE_MULTI_WAIT_SINGLE;
            break;

        case ARRIVE_MULTI_WAIT_SINGLE:
            confirmed = arrival_multi_phase_confirm(
                (s->lineNum == 1 || s->ledNum <= 3) ? 1u : 0u);
            if (confirmed)
                arrival_detector.multi_phase = ARRIVE_MULTI_WAIT_MULTI_AGAIN;
            break;

        case ARRIVE_MULTI_WAIT_MULTI_AGAIN:
            confirmed = arrival_multi_phase_confirm(
                (s->lineNum > 1 && s->ledNum >= 4) ? 1u : 0u);
            if (confirmed)
            {
                arrival_detector.multi_phase = ARRIVE_MULTI_WAIT_MULTI;
                return 1;
            }
            break;

        default:
            arrival_detector_reset();
            break;
        }
    }

    return 0;
}

static uint8_t arrival_detector_update(volatile SCANER *s, u32 node_flag)
{
    u32 simple_flags = node_flag & ~(MUL2SING | MUL2MUL);

    if (deal_arrive(s, simple_flags))
    {
        if (arrival_detector.simple_count < ARRIVE_CONFIRM_SAMPLES)
            arrival_detector.simple_count++;
    }
    else
        arrival_detector.simple_count = 0;

    if (arrival_detector.simple_count >= ARRIVE_CONFIRM_SAMPLES)
    {
        arrival_detector.simple_count = 0;
        return 1;
    }

    return arrival_multiline_update(s, node_flag);
}

static uint8_t route_has_temp_track(u32 flag)
{
    return ((flag & (Temp_L | Temp_R | Temp_LiuShui)) != 0u) ? 1u : 0u;
}

static void temp_track_reset(u32 flag)
{
    temp_track_phase = route_has_temp_track(flag) ? TEMP_TRACK_PRIMARY : TEMP_TRACK_FINAL;
    temp_switch_mileage = 0.0f;
}

static void apply_temp_track_mode(u32 flag)
{
    uint8_t mode = LEFT_RIGHT_LINE;

    if ((flag & Temp_L) == Temp_L)
        mode = LEFT_LINE_MODE;
    else if ((flag & Temp_R) == Temp_R)
        mode = RIGHT_LINE_MODE;
    else if ((flag & Temp_LiuShui) == Temp_LiuShui)
        mode = CENTER_LINE_MODE;

    Line_SetTrackModeBumpless(mode);
}

static uint8_t temp_track_clearance_done(void)
{
    if (temp_track_phase != TEMP_TRACK_CLEARANCE)
        return 1;

    if (fabsf(Chassis_GetMileage() - temp_switch_mileage) < TEMP_TRACK_CLEAR_CM)
        return 0;

    temp_track_phase = TEMP_TRACK_FINAL;
    arrival_detector_reset();
    return 1;
}

static void route_phase_reset(void)
{
    route_state = CROSS_ROUTE_INIT;
    is_near_end = 0;
    detect_started = 0;
    yaw_reset_applied = 0;
    arrival_detector_reset();
    temp_track_reset(0);
}

/**
 * @brief  重置 Cross 状态机（由 mapInit 调用）
 */
void Cross_reset(void)
{
    route_phase_reset();
}

static void cross_line_init(void)
{
    Chassis_ClearMileage();
    arrival_detector_reset();
    temp_track_reset(nodesr.nowNode.flag);
    if (route_is_p2_to_n2())
        Line_SetTrackModeBumpless(CENTER_LINE_MODE);
    else
        Line_SetTrackModeBumpless(track_mode_from_flag(nodesr.nowNode.flag));
    detect_started = 0;
    yaw_reset_applied = 0;
    route_state = CROSS_ROUTE_CONFIGURED;
}

static void cross_line_start(void)
{
    Chassis_SetMode(is_Line);
    Chassis_SetTargetSpeed(nodesr.nowNode.speed);
    route_state = CROSS_ROUTE_TRACKING;
}

static void cross_track_switch(void)
{
    if (!route_is_p2_to_n2())
        return;
    if (route_state != CROSS_ROUTE_TRACKING)
        return;
    if (fabsf(Chassis_GetMileage()) < ROUTE_HALF_RATIO * nodesr.nowNode.step)
        return;

    Line_SetTrackModeBumpless(RIGHT_LINE_MODE);
    route_state = CROSS_ROUTE_P2_RIGHT;
}

static void cross_detect_start(void)
{
    if (!detect_started &&
        fabsf(Chassis_GetMileage()) >= ROUTE_DETECT_RATIO * nodesr.nowNode.step)
    {
        detect_started = 1;
    }
}

static void cross_limit_speed(float max_speed)
{
    if (motor_all.Cspeed > max_speed)
        Chassis_SetTargetSpeed(max_speed);
}

static void cross_apply_approach_speed(void)
{
    float ad;
    float ad2;

    if (fabsf(Chassis_GetMileage()) < ROUTE_SLOW_RATIO * nodesr.nowNode.step)
        return;

    ad  = fabsf(need2turn(getAngleZ(), nodesr.nextNode.angle));
    ad2 = fabsf(need2turn(nodesr.nowNode.angle, nodesr.nextNode.angle));

    if (!yaw_reset_applied &&
        (nodesr.nowNode.flag & RESTMPUZ) == RESTMPUZ &&
        (Scaner.detail & SCANER_CENTER_MASK) == SCANER_CENTER_MASK)
    {
        mpuZreset(imu.yaw, nodesr.nowNode.angle);
        yaw_reset_applied = 1u;
    }

    if ((nodesr.nowNode.flag & SLOWDOWN) == SLOWDOWN)
        cross_limit_speed(SPEED0);
    else if (route_need_turn(ad, ad2))
        cross_limit_speed(SPEED1);
    else if (nodesr.nextNode.speed < nodesr.nowNode.speed)
        cross_limit_speed(nodesr.nextNode.speed);
}

static void cross_arrive_check(void)
{
    if (!detect_started || route_arrived())
        return;

    if (!temp_track_clearance_done())
        return;

    /* P1→N1：25cm前屏蔽到达检测，25cm后开放（角度180区分来路） */
    if (nodesr.nowNode.nodenum == N1 && nodesr.nowNode.angle == 180.0f)
    {
        if (fabsf(Chassis_GetMileage()) < 25.0f)
            return;
    }

    getline_error();
    if (arrival_detector_update(&Scaner, nodesr.nowNode.flag))
    {
        if (temp_track_phase == TEMP_TRACK_PRIMARY)
        {
            apply_temp_track_mode(nodesr.nowNode.flag);
            temp_track_phase = TEMP_TRACK_CLEARANCE;
            temp_switch_mileage = Chassis_GetMileage();
            arrival_detector_reset();
            return;
        }

        route_set_arrived();
    }

    if (!route_arrived() && nodesr.nowNode.step <= 1u)
    {
        route_set_arrived();
    }

    if (!route_arrived() && (nodesr.nowNode.flag & INGNORE) == INGNORE &&
        fabsf(Chassis_GetMileage()) >= nodesr.nowNode.step)
    {
        route_set_arrived();
    }

    if (!route_arrived() &&
        fabsf(Chassis_GetMileage()) >= nodesr.nowNode.step * ROUTE_SEARCH_RATIO)
    {
        cross_limit_speed(SPEED0);
    }

    if (!route_arrived() &&
        fabsf(Chassis_GetMileage()) >= nodesr.nowNode.step * ROUTE_FAULT_RATIO)
    {
        Chassis_ForceStop(CHASSIS_STOP_ROUTE_INVALID);
        return;
    }

    /* 坡道保护：未到节点但pitch已变，强制到达防冲坡（N1/N4下坡后pitch不稳，排除） */
    if (!route_arrived() && nodesr.nowNode.function == NONE &&
        !(nodesr.nowNode.nodenum == N1 && nodesr.nowNode.angle == 180.0f) &&
        nodesr.nowNode.nodenum != N4 &&
        fabsf(imu.pitch - basic_p) > 5.0f)
    {
        route_set_arrived();
    }
}

static void cross_line_update(void)
{
    if (route_state == CROSS_ROUTE_INIT)
        cross_line_init();

    if (route_state == CROSS_ROUTE_CONFIGURED)
        cross_line_start();

    cross_track_switch();
    cross_detect_start();
    cross_apply_approach_speed();
    cross_arrive_check();

    if (route_arrived())
        is_near_end = 1;
}

static void cross_barrier_update(void)
{
    MapPostTurnAction_t post_turn;

    post_turn = map_function(nodesr.nowNode.function);

    if (Chassis_IsStopLocked())
        return;

    if (post_turn == MAP_POST_TURN_SKIP && route_arrived())
    {
        route_clear_arrived();
        route_phase_reset();
        cross_node_advance();
        return;
    }

    route_phase_reset();
}

static uint8_t cross_action_succeeded(ChassisActionResult_t result)
{
    if (result == CHASSIS_ACTION_OK)
        return 1u;

    if (!Chassis_IsStopLocked())
        Chassis_ForceStop(CHASSIS_STOP_MOTION_TIMEOUT);
    return 0u;
}

static uint8_t cross_stop_turn(void)
{
    if (!cross_action_succeeded(
            Chassis_DriveDistance_Timeout(is_Gyro, 15.0f, SPEED1,
                                          getAngleZ(),
                                          MAP_CLEARANCE_TIMEOUT_MS)))
    {
        return 0u;
    }

    CarBrake();
    vTaskDelay(STOP_TURN_SETTLE_MS);
    return cross_action_succeeded(
        Chassis_TurnTo_Timeout(nodesr.nextNode.angle, getAngleZ(),
                               MAP_STOP_TURN_TIMEOUT_MS));
}

static uint8_t cross_run_turn(void)
{
    float err;
    uint16_t timeout;
    uint8_t  was_near;  /* 曾经接近过目标 */
    uint8_t  turn_failed = 0u;

    float old_speed_max;
    float old_kd;

    if (!cross_action_succeeded(
            Chassis_DriveDistance_Timeout(is_Gyro, 5.0f, SPEED1,
                                          getAngleZ(),
                                          MAP_CLEARANCE_TIMEOUT_MS)))
    {
        return 0u;
    }

    /* 限幅差速 + 提高阻尼，防止暴力旋转和来回振荡 */
    old_speed_max = motor_all.GyroT_speedMax;
    old_kd = gyroT_pid_param.kd;
    motor_all.GyroT_speedMax = TURN_RUN_SPEED_MAX;
    gyroT_pid_param.kd = TURN_RUN_KD_BOOST;

    Chassis_SetMode(is_Turn);
    angle.AngleT = nodesr.nextNode.angle;

    err      = fabsf(need2turn(getAngleZ(), nodesr.nextNode.angle));
    was_near = 0;
    timeout  = TURN_TIMEOUT_CYCLES;

    while (err > TURN_DONE_DEADBAND && !Chassis_IsStopLocked())
    {
        vTaskDelay(CONTROL_CYCLE_MS);
        err = fabsf(need2turn(getAngleZ(), nodesr.nextNode.angle));

        /* 曾经接近目标(8°内)，现在又弹回超过20°，按转向失败处理。 */
        if (err < TURN_OSCILLATE_NEAR)
            was_near = 1;
        if (was_near && err > TURN_OSCILLATE_FAR)
        {
            turn_failed = 1u;
            break;
        }

        /* 硬超时兜底 */
        if (--timeout == 0)
        {
            turn_failed = 1u;
            break;
        }
    }

    motor_all.GyroT_speedMax = old_speed_max;
    gyroT_pid_param.kd = old_kd;

    if (Chassis_IsStopLocked())
        return 0u;
    if (turn_failed)
    {
        Chassis_ForceStop(CHASSIS_STOP_MOTION_TIMEOUT);
        return 0u;
    }
    return 1u;
}

static uint8_t cross_need_gyro_clearance(void)
{
    uint8_t needs_clearance;

    needs_clearance =
        (nodesr.nowNode.nodenum == N4 && nodesr.nextNode.nodenum == N3) ||
        (nodesr.nowNode.nodenum == N3 && nodesr.nextNode.nodenum == P3);

    if (!needs_clearance)
        return 1;

    return cross_action_succeeded(
        Chassis_DriveDistance_Timeout(is_Gyro, 20.0f,
                                      nodesr.nextNode.speed, getAngleZ(),
                                      MAP_CLEARANCE_TIMEOUT_MS));
}

static uint8_t cross_special_n2_b1(void)
{
    if (nodesr.lastNode.nodenum != P2 ||
        nodesr.nowNode.nodenum != N2 ||
        nodesr.nextNode.nodenum != B1)
    {
        return 1u;
    }

    mpuZreset(imu.yaw, nodesr.nowNode.angle);
    if (!cross_action_succeeded(
            Chassis_DriveDistance_Timeout(is_Gyro, N2_B1_PASS_CM, SPEED1,
                                          nodesr.nowNode.angle,
                                          MAP_CLEARANCE_TIMEOUT_MS)))
    {
        return 0u;
    }
    Line_SetTrackModeBumpless(CENTER_LINE_MODE);
    return 1u;
}

static uint8_t cross_route_end(void)
{
    if (route[map.point] != ROUTE_END)
        return 0;

    CarBrake();
    map.routetime += 1;
    return 1;
}

static void cross_node_advance(void)
{
    nodesr.lastNode = nodesr.nowNode;
    nodesr.nowNode = nodesr.nextNode;

    if (cross_route_end())
        return;

    if (!map_load_next_node(nodesr.nowNode.nodenum, route[map.point]))
        return;
    map.point++;
    if (!cross_special_n2_b1())
        return;

    cross_line_init();
    cross_line_start();
}

static void cross_turn_update(void)
{
    float ad;
    float ad2;
    uint8_t turn_ok;

    if (!route_arrived())
        return;

    route_clear_arrived();

    ad  = fabsf(need2turn(getAngleZ(), nodesr.nextNode.angle));
    ad2 = fabsf(need2turn(nodesr.nowNode.angle, nodesr.nextNode.angle));

    if (!route_need_turn(ad, ad2))
        turn_ok = cross_need_gyro_clearance();
    else if ((nodesr.nowNode.flag & DRIFT) == DRIFT)
        turn_ok = cross_run_turn();
    else if ((nodesr.nowNode.flag & L_follow) == L_follow)
    {
        Line_SetTrackModeBumpless(LEFT_LINE_MODE);
        turn_ok = cross_run_turn();
    }
    else if ((nodesr.nowNode.flag & R_follow) == R_follow)
    {
        Line_SetTrackModeBumpless(RIGHT_LINE_MODE);
        turn_ok = cross_run_turn();
    }
    else if ((nodesr.nowNode.flag & STOPTURN) == STOPTURN || ad > TURN_STOP_ANGLE)
        turn_ok = cross_stop_turn();
    else
        turn_ok = cross_run_turn();

    if (turn_ok)
        cross_node_advance();
}

/**
 * @brief  Cross 状态机 - 节点间处理核心
 * @details 所有运动通过 chassis_api 控制。
 *          状态流程：
 *          1. 路径初始化 (route_state=0): 清零里程并设置循线模式
 *          2. 持续巡线 (route_state=1→2): 设速度，循线前进
 *          3. Temp模式切换: 首次确认岔口后切换，清出10cm再恢复到达检测
 *          4. P2专用切换 (route_state=2→3): 50%时由居中改为右循线
 *          5. 到达检测 → 障碍物处理 → 转弯 → 节点切换
 */
void Cross(void)
{
    if (Chassis_IsStopLocked())
        return;

    if (is_near_end == 0)
        cross_line_update();
    else if (is_near_end == 1)
        cross_barrier_update();

    if (Chassis_IsStopLocked())
        return;

    cross_turn_update();
}
