/**
 * @file    map.c
 * @brief   地图管理和Cross状态机模块
 * @details 包含地图初始化、Cross节点间处理状态机、到达判断、障碍物分发
 *          参考 xunbao 的架构：所有运动通过 chassis_api 控制，含巡线保护。
 */

#include "map.h"
#include "route_builder.h"
#include "route_catalog.h"
#include "../chassis/chassis_api.h"
#include "../barrier/barrier.h"
#include "scaner.h"
#include "motor_task.h"
#include "imu.h"
#include "delay.h"
#include "math.h"
#include "bsp_linefollower.h"
#include "pid.h"
#include "vision_api.h"

/* ======================== 控制周期和延时常量 ======================== */

#define CONTROL_CYCLE_MS        5       /* 控制周期 5ms */
#define DELAY_SHORT             100     /* 短暂等待 */
#define N2_B1_PASS_CM           10.0f
#define NODE_ARRIVED_FLAG       0x04u
#define LEFT_LINE_MODE          1
#define RIGHT_LINE_MODE         2
#define CENTER_LINE_MODE        3
#define SCANER_LEFT_BRANCH_MASK 0x0180u  /* 中间偏左两路循迹灯 */
#define SCANER_RIGHT_BRANCH_MASK 0x0018u /* 中间偏右两路循迹灯 */
#define ROUTE_HALF_RATIO        0.5f
#define ROUTE_DETECT_RATIO      0.3f
#define ROUTE_SLOW_RATIO        0.7f
#define ROUTE_SEARCH_RATIO      1.15f
#define ROUTE_FAULT_RATIO       1.40f
#define TURN_NEED_ANGLE         10.0f
#define TURN_STOP_ANGLE         90.0f
#define TURN_DONE_DEADBAND      3.0f
#define MAP_DRIVE_TIMEOUT_MS    6000u
#define MAP_TURN_TIMEOUT_MS     8000u

/* ======================== 全局变量定义 ======================== */

struct Map_State map = {0, 0};
NODESR nodesr;
uint8_t isAllRoute = 1;

/* 第一轮基础段；到 N12 后由 DOOR 视觉结果提交后续路线片段。 */
u8 route[100] = {N2, B1, N1, P1, N1, B2, N4, N5, N6, P4, N6,
                 N5, N12, ROUTE_END};

static uint8_t map_load_next_node(void)
{
    uint8_t connection;

    if (map.point >= ROUTE_CAPACITY || route[map.point] == ROUTE_END)
        return 0u;
    connection = getNextConnectNode(nodesr.nowNode.nodenum, route[map.point]);
    if (connection == MAP_NODE_INDEX_INVALID)
    {
        Chassis_ForceStop(CHASSIS_STOP_ROUTE_INVALID);
        return 0u;
    }
    nodesr.nextNode = Node[connection];
    map.point++;
    return 1u;
}

uint8_t Map_ReloadRouteFromCurrent(void)
{
    map.point = 0u;
    if (!Map_ValidateRoute(nodesr.nowNode.nodenum))
    {
        Chassis_ForceStop(CHASSIS_STOP_ROUTE_INVALID);
        return 0u;
    }
    return map_load_next_node();
}

/* ======================== 底层驱动封装 ======================== */

/**
 * @brief  设置循线模式（左循线/右循线/流水灯）
 * @param  flag 节点标志位
 */
static void SetTrackMode(u32 flag)
{
    if ((flag & LEFT_LINE) == LEFT_LINE)
        LEFT_RIGHT_LINE = LEFT_LINE_MODE;
    else if ((flag & RIGHT_LINE) == RIGHT_LINE)
        LEFT_RIGHT_LINE = RIGHT_LINE_MODE;
    else if ((flag & LiuShui) == LiuShui)
        LEFT_RIGHT_LINE = CENTER_LINE_MODE;
    else
        LEFT_RIGHT_LINE = 0;
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
    Chassis_EnableStallProtection();

    if (!Map_ValidateData())
    {
        Chassis_ForceStop(CHASSIS_STOP_ROUTE_INVALID);
        return;
    }

    /* 起点：P2平台 */
    nodesr.nowNode.nodenum = P2;
    nodesr.nowNode.angle = 0;
    nodesr.nowNode.function = NONE;
    nodesr.nowNode.speed = SPEED1;
    nodesr.nowNode.step = 10;
    nodesr.nowNode.flag = CLEFT | RIGHT_LINE;

    if (!Map_ValidateRoute(nodesr.nowNode.nodenum))
    {
        Chassis_ForceStop(CHASSIS_STOP_ROUTE_INVALID);
        return;
    }

    /* 获取第一个目标节点 */
    (void)map_load_next_node();
}

void mapInit1(void)
{
    uint8_t start_node = nodesr.nowNode.nodenum;

    if (start_node >= MAP_NODE_COUNT)
        start_node = P2;
    map.point = 0;
    map.routetime = 1;
    nodesr.flag = 0;
    Cross_reset();
    Chassis_ClearMileage();
    Chassis_ClearStopLock();
    Chassis_EnableRollProtection();
    Chassis_EnableYawJumpProtection();
    Chassis_EnableStallProtection();
    Vision_ClearResults();

    nodesr.lastNode.nodenum = start_node;
    nodesr.nowNode.nodenum = start_node;
    nodesr.nowNode.angle = getAngleZ();
    nodesr.nowNode.function = NONE;
    nodesr.nowNode.speed = SPEED0;
    nodesr.nowNode.step = 1;
    nodesr.nowNode.flag = NO | NOTURN;
    mpuZreset(imu.yaw, nodesr.nowNode.angle);
    (void)Map_ReloadRouteFromCurrent();
}

/* ======================== 节点连接查找 ======================== */

u8 getNextConnectNode(u8 nownode, u8 nextnode)
{
    if (nownode >= MAP_NODE_COUNT || nextnode >= MAP_NODE_COUNT)
        return MAP_NODE_INDEX_INVALID;

    unsigned char rest = ConnectionNum[nownode];
    unsigned char addr = Address[nownode];

    for (int i = 0; i < rest; i++)
    {
        if (Node[addr].nodenum == nextnode)
            return addr;
        addr++;
    }
    return MAP_NODE_INDEX_INVALID;
}

uint8_t Map_ValidateData(void)
{
    uint8_t node;
    uint8_t i;
    static const uint8_t allowed_one_way[][2] = {
        {B1, P2}, {N2, B1}, {N10, N12}, {N12, P6},
        {N13, N18}, {B4, C5}, {B4, N18}, {N19, N13}
    };

    if (Address[0] != 0u || Address[MAP_NODE_COUNT] != MAP_CONNECTION_COUNT)
        return 0;

    for (node = 0; node < MAP_NODE_COUNT; node++)
    {
        if ((uint16_t)Address[node] + ConnectionNum[node] != Address[node + 1u])
            return 0;
        for (i = 0; i < ConnectionNum[node]; i++)
        {
            uint8_t index = (uint8_t)(Address[node] + i);
            uint8_t reverse_missing;
            uint8_t allowed = 0u;
            uint8_t exception;
            if (index >= MAP_CONNECTION_COUNT || Node[index].nodenum >= MAP_NODE_COUNT)
                return 0;
            if (Node[index].step == 0u && (Node[index].flag & NO) == 0u &&
                Node[index].function != UNDER &&
                !(node == N19 && Node[index].nodenum == N13))
                return 0u;
            reverse_missing = getNextConnectNode(Node[index].nodenum, node) ==
                              MAP_NODE_INDEX_INVALID;
            if (reverse_missing)
            {
                for (exception = 0u;
                     exception < (uint8_t)(sizeof(allowed_one_way) / sizeof(allowed_one_way[0]));
                     exception++)
                {
                    if (allowed_one_way[exception][0] == node &&
                        allowed_one_way[exception][1] == Node[index].nodenum)
                    {
                        allowed = 1u;
                        break;
                    }
                }
                if (!allowed)
                    return 0u;
            }
        }
    }
    return RouteCatalog_ValidateAll();
}

uint8_t Map_ValidateRoute(uint8_t start_node)
{
    uint8_t current = start_node;
    uint8_t i;

    if (start_node >= MAP_NODE_COUNT)
        return 0u;
    for (i = 0u; i < ROUTE_CAPACITY; i++)
    {
        if (route[i] == ROUTE_END)
            return i > 0u ? 1u : 0u;
        if (getNextConnectNode(current, route[i]) == MAP_NODE_INDEX_INVALID)
            return 0u;
        current = route[i];
    }
    return 0u;
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
    ChassisActionResult_t result = CHASSIS_ACTION_OK;

    switch (fun)
    {
        case NONE:
            break;
        case UpStage:
            Stage();                             /* 通用平台（P1/P3/P4等） */
            return MAP_POST_TURN_SKIP;
        case Bridge:
            Barrier_Bridge();                    /* 过桥 */
            break;
        case Hill:
            Barrier_Hill();                      /* 楼梯 */
            break;
        case LBHill:
            result = Barrier_DoubleHill();
            break;
        case SM:
            result = Barrier_SwordMountain();
            break;
        case View:
            result = Barrier_View(1u);
            break;
        case View1:
            result = Barrier_View(0u);
            break;
        case BACK:
            result = Barrier_Back();
            break;
        case BSoutPole:
            result = Barrier_SouthPole();
            break;
        case QQB:
            result = Barrier_Seesaw();
            break;
        case BLBS:
            Barrier_WavedPlate(87.0f);
            break;
        case BLBL:
            Barrier_WavedPlate(170.0f);
            break;
        case DOOR:
            result = Barrier_Door();
            break;
        case BHM:
            result = Barrier_HighMountain();
            break;
        case IGNORE:
            break;
        case UNDER:
            result = Barrier_Under();
            break;
        case Special_node:
            result = Barrier_SpecialNode();
            break;
        case DOOR1:
            result = Barrier_Door();
            break;
        case UpStageP2:
            Stage_P2();                          /* P2平台 */
            return MAP_POST_TURN_SKIP;
        default:
            break;
    }

    if (result != CHASSIS_ACTION_OK && !Chassis_IsStopLocked())
        Chassis_ForceStop(CHASSIS_STOP_BARRIER_FAILED);

    return MAP_POST_TURN_NORMAL;
}

/* ======================== Cross 状态机 ======================== */

/* Cross 状态机内部状态（由 Cross_reset 重置） */
static uint8_t route_state = 0;
static uint8_t is_near_end = 0;
static uint8_t detect_started = 0;
static uint8_t yaw_reset_done = 0;
static uint8_t follow_pid_saved = 0;
static struct PID_param saved_line_pid;

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

static void route_phase_reset(void)
{
    if (follow_pid_saved)
    {
        line_pid_param = saved_line_pid;
        follow_pid_saved = 0;
    }
    route_state = 0;
    is_near_end = 0;
    detect_started = 0;
    yaw_reset_done = 0;
}

static void cross_line_protect_on(void)
{
    Chassis_EnableAntiSnake();
    Chassis_EnableLineLostProtection();
}

static void cross_line_protect_off(void)
{
    Chassis_DisableAntiSnake();
    Chassis_DisableLineLostProtection();
}

/**
 * @brief  重置 Cross 状态机（由 mapInit 调用）
 */
void Cross_reset(void)
{
    route_phase_reset();
    cross_line_protect_off();
}

static void cross_line_init(void)
{
    Chassis_ClearMileage();
    if (route_is_p2_to_n2())
        LEFT_RIGHT_LINE = CENTER_LINE_MODE;
    else
        SetTrackMode(nodesr.nowNode.flag);
    detect_started = 0;
    route_state = 1;
}

static void cross_line_start(void)
{
    Chassis_SetTargetSpeed(nodesr.nowNode.speed);
    Chassis_SetMode(is_Line);
    cross_line_protect_on();
    route_state = 2;
}

static void cross_track_switch(void)
{
    if (route_state != 2)
        return;
    if (fabsf(Chassis_GetMileage()) < ROUTE_HALF_RATIO * nodesr.nowNode.step)
        return;

    if (route_is_p2_to_n2())
        LEFT_RIGHT_LINE = RIGHT_LINE_MODE;
    else if ((nodesr.nowNode.flag & Temp_L) == Temp_L)
        LEFT_RIGHT_LINE = LEFT_LINE_MODE;
    else if ((nodesr.nowNode.flag & Temp_R) == Temp_R)
        LEFT_RIGHT_LINE = RIGHT_LINE_MODE;
    else if ((nodesr.nowNode.flag & Temp_LiuShui) == Temp_LiuShui)
        LEFT_RIGHT_LINE = CENTER_LINE_MODE;

    route_state = 3;
}

static void cross_detect_start(void)
{
    if (!detect_started &&
        fabsf(Chassis_GetMileage()) >= ROUTE_DETECT_RATIO * nodesr.nowNode.step)
    {
        detect_started = 1;
        if ((nodesr.nowNode.flag & RESTMPUZ) == RESTMPUZ &&
            !yaw_reset_done && Scaner.ledNum > 0u)
        {
            mpuZreset(imu.yaw, nodesr.nowNode.angle);
            yaw_reset_done = 1;
        }
    }
}

uint8_t Cross_GetState(void)
{
    return route_state;
}

static void cross_arrive_slowdown(void)
{
    float ad;
    float ad2;

    if (fabsf(Chassis_GetMileage()) < ROUTE_SLOW_RATIO * nodesr.nowNode.step)
        return;

    ad  = fabsf(need2turn(getAngleZ(), nodesr.nextNode.angle));
    ad2 = fabsf(need2turn(nodesr.nowNode.angle, nodesr.nextNode.angle));

    if ((nodesr.nowNode.flag & SLOWDOWN) == SLOWDOWN)
    {
        Chassis_SetTargetSpeed(SPEED0);
    }
    else if (route_need_turn(ad, ad2))
    {
        Chassis_SetTargetSpeed(SPEED1);
    }
    else if (nodesr.nextNode.speed < nodesr.nowNode.speed)
    {
        Chassis_SetTargetSpeed(nodesr.nextNode.speed);
    }
}

static void cross_arrive_check(void)
{
    float travelled;

    if (!detect_started || route_arrived())
        return;

    travelled = fabsf(Chassis_GetMileage());

    if (nodesr.nowNode.step == 0u)
    {
        route_set_arrived();
        return;
    }
    if ((nodesr.nowNode.flag & (NO | INGNORE)) != 0u &&
        travelled >= (float)nodesr.nowNode.step)
    {
        route_set_arrived();
        return;
    }

    getline_error();
    if (deal_arrive(&Scaner, nodesr.nowNode.flag))
    {
        route_set_arrived();
        cross_arrive_slowdown();
        return;
    }

    if (travelled >= ROUTE_FAULT_RATIO * (float)nodesr.nowNode.step)
    {
        Chassis_ForceStop(CHASSIS_STOP_ROUTE_INVALID);
        return;
    }
    if (travelled >= ROUTE_SEARCH_RATIO * (float)nodesr.nowNode.step)
        Chassis_SetTargetSpeed(SPEED0);
}

static void cross_line_update(void)
{
    if (route_state == 0)
        cross_line_init();

    if (route_state == 1)
        cross_line_start();

    cross_track_switch();
    cross_detect_start();
    cross_arrive_slowdown();

    if (!follow_pid_saved &&
        (nodesr.nowNode.flag & (L_follow | R_follow)) != 0u)
    {
        saved_line_pid = line_pid_param;
        follow_pid_saved = 1u;
        if ((nodesr.nowNode.flag & L_follow) == L_follow)
            LEFT_RIGHT_LINE = LEFT_LINE_MODE;
        else if ((nodesr.nowNode.flag & R_follow) == R_follow)
            LEFT_RIGHT_LINE = RIGHT_LINE_MODE;
        line_pid_param.kp *= 1.35f;
        line_pid_param.kd *= 1.20f;
    }
    cross_arrive_check();

    if (route_arrived())
        is_near_end = 1;
}

static void cross_barrier_update(void)
{
    MapPostTurnAction_t post_turn;

    cross_line_protect_off();
    post_turn = map_function(nodesr.nowNode.function);

    if (post_turn == MAP_POST_TURN_SKIP && route_arrived())
    {
        route_clear_arrived();
        route_phase_reset();
        cross_node_advance();
        return;
    }

    route_phase_reset();
}

static void cross_pass_turn(void)
{
    Chassis_ClearMileage();
    Chassis_SetTargetSpeed(nodesr.nextNode.speed);
    Chassis_SetMode(is_Line);
}

static void cross_stop_turn(void)
{
    (void)Chassis_DriveDistance_Blocking(is_Gyro, 15.0f, SPEED1,
                                         getAngleZ(), MAP_DRIVE_TIMEOUT_MS);
    CarBrake();
    vTaskDelay(DELAY_SHORT);
    (void)Chassis_Turn_By_StopGyro_Blocking(nodesr.nextNode.angle,
                                            getAngleZ(), MAP_TURN_TIMEOUT_MS);
}

static void cross_run_turn(void)
{
    TickType_t started;
    (void)Chassis_DriveDistance_Blocking(is_Gyro, 5.0f, SPEED1,
                                         getAngleZ(), MAP_DRIVE_TIMEOUT_MS);

    Chassis_SetMode(is_Turn);
    angle.AngleT = nodesr.nextNode.angle;
    started = xTaskGetTickCount();
    while (fabsf(need2turn(getAngleZ(), nodesr.nextNode.angle)) > TURN_DONE_DEADBAND)
    {
        if ((uint32_t)((xTaskGetTickCount() - started) * portTICK_PERIOD_MS) >=
            MAP_TURN_TIMEOUT_MS)
        {
            Chassis_ForceStop(CHASSIS_STOP_MOTION_TIMEOUT);
            return;
        }
        vTaskDelay(CONTROL_CYCLE_MS);
    }
}

static void cross_drift_turn(void)
{
    /* Keep the chassis rolling into the gyro turn; the exit is bounded by yaw. */
    (void)Chassis_DriveDistance_Blocking(is_Gyro, 8.0f, SPEED1,
                                         getAngleZ(), MAP_DRIVE_TIMEOUT_MS);
    cross_run_turn();
    if (!Chassis_IsStopLocked())
    {
        Chassis_SetMode(is_Line);
        Chassis_SetTargetSpeed(nodesr.nextNode.speed);
    }
}

static void cross_special_n2_b1(void)
{
    if (nodesr.lastNode.nodenum != P2 ||
        nodesr.nowNode.nodenum != N2 ||
        nodesr.nextNode.nodenum != B1)
    {
        return;
    }

    mpuZreset(imu.yaw, nodesr.nowNode.angle);
    (void)Chassis_DriveDistance_Blocking(is_Gyro, N2_B1_PASS_CM, SPEED1,
                                         nodesr.nowNode.angle, MAP_DRIVE_TIMEOUT_MS);
    LEFT_RIGHT_LINE = CENTER_LINE_MODE;
}

static uint8_t cross_route_end(void)
{
    if (map.point >= ROUTE_CAPACITY)
    {
        Chassis_ForceStop(CHASSIS_STOP_ROUTE_INVALID);
        return 1;
    }
    if (route[map.point] != ROUTE_END)
        return 0;

    cross_line_protect_off();
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

    if (!map_load_next_node())
        return;
    cross_special_n2_b1();

    Chassis_ClearMileage();
    Chassis_SetTargetSpeed(nodesr.nowNode.speed);
    Chassis_SetMode(is_Line);
    cross_line_protect_on();
}

static void cross_turn_update(void)
{
    float ad;
    float ad2;

    if (!route_arrived())
        return;

    route_clear_arrived();
    cross_line_protect_off();

    ad  = fabsf(need2turn(getAngleZ(), nodesr.nextNode.angle));
    ad2 = fabsf(need2turn(nodesr.nowNode.angle, nodesr.nextNode.angle));

    if (!route_need_turn(ad, ad2))
        cross_pass_turn();
    else if ((nodesr.nowNode.flag & DRIFT) == DRIFT)
        cross_drift_turn();
    else if ((nodesr.nowNode.flag & STOPTURN) == STOPTURN || ad > TURN_STOP_ANGLE)
        cross_stop_turn();
    else
        cross_run_turn();

    cross_node_advance();
}

/**
 * @brief  Cross 状态机 - 节点间处理核心
 * @details 所有运动通过 chassis_api 控制（参考 xunbao 架构）。
 *          状态流程：
 *          1. 路径初始化 (route_state=0): 清零里程，设置巡线模式，使能巡线保护
 *          2. 前半段巡线 (route_state=1→2): 设速度，循线前进
 *          3. 模式切换 (route_state=2→3): 50%时切换巡线模式
 *          4. 减速判断 (route_state=3): 70%时根据角度差决定是否减速
 *          5. 到达检测 → 障碍物处理 → 转弯 → 节点切换
 */
void Cross(void)
{
    if (is_near_end == 0)
        cross_line_update();
    else if (is_near_end == 1)
        cross_barrier_update();

    cross_turn_update();
}
