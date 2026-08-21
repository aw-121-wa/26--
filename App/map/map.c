/**
 * @file    map.c
 * @brief   地图管理和Cross状态机模块
 * @details 包含地图初始化、Cross节点间处理状态机、到达判断、障碍物分发
 *          参考 xunbao 的架构：所有运动通过 chassis_api 控制，含巡线保护。
 */

#include "map.h"
#include "../chassis/chassis_api.h"
#include "../barrier/barrier.h"
#include "scaner.h"
#include "motor_task.h"
#include "imu.h"
#include "delay.h"
#include "hmi_display.h"
#include "math.h"
#include "bsp_linefollower.h"
#include "stdio.h"
#include "usart.h"

/* ======================== 控制周期和延时常量 ======================== */

#define CONTROL_CYCLE_MS        5       /* 控制周期 5ms */
#define DELAY_SHORT             100     /* 短暂等待 */
#define N2_B1_PASS_CM           10.0f
#define P3_N3_TURN_FORWARD_CM   25.0f
#define P3_N3_POST_TURN_FORWARD_CM 8.0f
#define P2_N2_STOP_MS           50     /* P2到达N2后停车延时(ms) */
#define NODE_ARRIVED_FLAG       0x04u
#define LEFT_LINE_MODE          1
#define RIGHT_LINE_MODE         2
#define CENTER_LINE_MODE        3
#define SCANER_LEFT_BRANCH_MASK 0x0180u  /* 中间偏左两路循迹灯 */
#define SCANER_RIGHT_BRANCH_MASK 0x0018u /* 中间偏右两路循迹灯 */
#define ROUTE_HALF_RATIO        0.5f
#define ROUTE_DETECT_RATIO      0.7f    /* 与参考工程一致：后 30% 才启动到达检测 */
#define ROUTE_SLOW_RATIO        0.7f    /* 70%里程开始减速，与检测窗口同步 */
#define ROUTE_FORK_START        0.25f   /* 25%里程开始岔口主动防护 */
#define ROUTE_FORK_END          0.80f   /* 80%里程结束岔口主动防护 */
#define ARRIVE_CONFIRM_SAMPLES  3u
#define TEMP_TRACK_CLEAR_CM     10.0f
#define NODE_ARRIVAL_CLEAR_CM    5.0f    /* 到达节点后清出标记区 */
#define TURN_NEED_ANGLE         10.0f

/* ======================== 保护阈值 ======================== */

#define NODE_REENTRY_CM         10.0f    /* 节点重入保护距离(cm) */
#define ROUTE_FORCE_RATIO       1.0f    /* 里程超标强制到达阈值（100%段长） */
#define TURN_STOP_ANGLE         90.0f
#define TURN_SCALE              1.0f    /* 转弯比例补偿 */

/* ======================== 全局变量定义 ======================== */

struct Map_State map = {0, 0};
NODESR nodesr;

/* 第一轮路线：P2 -> N2 -> B1 -> N1 -> P1 -> ... -> P3 -> N3 -> D4 -> N8。
 * D4 到达时按门灯颜色动态改线：BLACK 换门至 D3，GREEN/BLUE 前向拼接去 P5 返回 P2；
 * 末尾保留 N8 作为 D4 BLACK→D3 换门的 Map_SpliceInsertDetour reentry。 */
u8 route[100] = {N2, B3, N4, N5, N6, P4, N6, N5, N4, N3, P3, N3, D4, N8, ROUTE_END};

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
    HmiDisplay_ResetScores();

    /* 起点：P2平台 */
    nodesr.nowNode.nodenum = P2;
    nodesr.nowNode.angle = 0;
    nodesr.nowNode.function = NONE;
    nodesr.nowNode.speed = SPEED1;
    nodesr.nowNode.step = 20;   /* 与 P2→N2 边 step 一致，避免 10cm 就提前里程强制到达 */
    nodesr.nowNode.flag = CLEFT | RIGHT_LINE;

    /* 获取第一个目标节点 */
    nodesr.nextNode = Node[getNextConnectNode(nodesr.nowNode.nodenum, route[map.point++])];
}

/* ======================== 第二轮开始 ======================== */

/**
 * @brief  第二轮开始：覆盖 route[] 为二轮路线并重建 P2 起点现场。
 * @details 不调用 mapInit()，避免 HmiDisplay_ResetScores() 重置第一轮比赛信息。
 *          只重建 route/map.point/nodesr/Cross 状态；P2 起点参数复用地图初始化。
 */
void Map_StartRound2(const uint8_t *round2_route)
{
    uint8_t i;

    CarBrake();

    /* 覆盖 route[] 为二轮路线，其余位置填 ROUTE_END */
    if (round2_route != 0)
    {
        for (i = 0u; i < ROUTE_CAPACITY; i++)
        {
            if (round2_route[i] == ROUTE_END || round2_route[i] >= MAP_NODE_COUNT)
            {
                route[i] = ROUTE_END;
                break;
            }
            route[i] = round2_route[i];
        }
        for (; i < ROUTE_CAPACITY; i++)
            route[i] = ROUTE_END;
    }

    /* 从 P2 重新出发：map.point/routetime/nodesr/Cross 全部清为第二轮起点态 */
    map.point = 0;
    map.routetime = 2;
    nodesr.flag = 0;
    Cross_reset();

    /* 起点：P2平台（与 mapInit 一致） */
    nodesr.lastNode.nodenum = 0;
    nodesr.nowNode.nodenum = P2;
    nodesr.nowNode.angle = 0;
    nodesr.nowNode.function = NONE;
    nodesr.nowNode.speed = SPEED1;
    nodesr.nowNode.step = 20;
    nodesr.nowNode.flag = CLEFT | RIGHT_LINE;
    nodesr.nextNode = Node[getNextConnectNode(nodesr.nowNode.nodenum, route[map.point++])];
}

/* ======================== 测试模式：从 N22 向 C10 出发 ======================== */

/**
 * @brief  测试模式地图初始化（跳过前段路线）
 * @details 复用主 route[]，把现场重建为"主程序中途运行到 N22、正驶向 C10"的状态：
 *          - lastNode  = C9→N22    （上一节点 N22）
 *          - nowNode   = N22→C10   （当前目标 C10，含 STOPTURN/BLBL）
 *          - nextNode  = C10→P7    （下一目标 P7，南极）
 *          - map.point = 28        （route[26]=C10 为当前目标，route[27]=P7 已预载进
 *            nextNode，后续从 route[28] 起继续消费，与主路线后半段一致）
 */
void mapInit_test_N22_C10(void)
{
    map.routetime = 0;
    map.point = 28;
    nodesr.flag = 0;
    Cross_reset();
    Chassis_EnableRollProtection();
    Chassis_EnableYawJumpProtection();
    HmiDisplay_ResetScores();

    nodesr.lastNode = Node[getNextConnectNode(C9, N22)];   /* C9→N22 */
    nodesr.nowNode  = Node[getNextConnectNode(N22, C10)];  /* N22→C10 */
    nodesr.nextNode = Node[getNextConnectNode(C10, P7)];   /* C10→P7（南极） */

    /* 对齐 IMU 航向基准：测试模式没有 zhunbei 的 mpuZreset 兜底，
       user_init 只把上电朝向标成 0°，而地图里 N22→C10 是 180°，
       必须把当前朝向显式归到 nowNode.angle，否则转弯/锁头会差 180° */
    mpuZreset(imu.yaw, nodesr.nowNode.angle);
}

/* ======================== 节点连接查找 ======================== */

u8 getNextConnectNode(u8 nownode, u8 nextnode)
{
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
    switch (fun)
    {
        case UpStage:
            Stage();                             /* 通用平台（P1/P3/P4等） */
            return MAP_POST_TURN_SKIP;
        case Bridge:
            Barrier_Bridge();                    /* 过桥 */
            break;
        case Hill:
            Barrier_Hill();                      /* 楼梯 */
            break;
        case BLBS:
            Barrier_WavedPlate(87.0f);
            break;
        case BLBL:
            Barrier_WavedPlate(140.0f);
            break;
        case DOOR:
        case DOOR1:
            Barrier_Door();
            break;
        case BSoutPole:
            Barrier_SouthPole();
            break;
        case BHM:
            Barrier_HighMountain();
            break;
        case UpStageP2:
            Stage_P2();                          /* P2平台 */
            return MAP_POST_TURN_SKIP;
        default:
            break;
    }

    return MAP_POST_TURN_NORMAL;
}

/* ======================== Cross 状态机 ======================== */

/* Cross 状态机内部状态（由 Cross_reset 重置） */
static uint8_t route_state = 0;
static uint8_t is_near_end = 0;
static uint8_t detect_started = 0;
static float  node_entry_mileage = 0.0f;  /* 节点切换时的里程（用于重入保护） */
static uint8_t route_last_segment = 0;    /* 已推进到最后一段，走完即结束一轮 */

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

static void cross_node_advance(void);

static uint8_t route_is_p2_to_n2(void)
{
    return (nodesr.nowNode.nodenum == P2 && nodesr.nextNode.nodenum == N2) ? 1 : 0;
}

static uint8_t route_arrived(void)
{
    return ((nodesr.flag & NODE_ARRIVED_FLAG) == NODE_ARRIVED_FLAG) ? 1 : 0;
}

uint8_t g_last_arrived_node = 0xFF;  /* 调试用：最近一次实际到达的节点编号 */

static void route_set_arrived(void)
{
    nodesr.flag |= NODE_ARRIVED_FLAG;
    g_last_arrived_node = nodesr.nowNode.nodenum;
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
    if ((flag & Temp_L) == Temp_L)
        LEFT_RIGHT_LINE = LEFT_LINE_MODE;
    else if ((flag & Temp_R) == Temp_R)
        LEFT_RIGHT_LINE = RIGHT_LINE_MODE;
    else if ((flag & Temp_LiuShui) == Temp_LiuShui)
        LEFT_RIGHT_LINE = CENTER_LINE_MODE;
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
    route_state = 0;
    is_near_end = 0;
    detect_started = 0;
    arrival_detector_reset();
    temp_track_reset(0);
}

uint8_t Cross_GetState(void)
{
    return route_state;
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
    route_last_segment = 0;
}

static RouteBuildStatus_t map_splice_fail(RouteBuildStatus_t status)
{
    Chassis_ForceStop(CHASSIS_STOP_ROUTE_INVALID);
    return status;
}

RouteBuildStatus_t Map_SpliceRemainingRoute(const uint8_t *segment)
{
    uint8_t current;
    uint8_t splice_index;
    uint8_t next_index;
    uint8_t length = 0u;
    uint8_t i;

    if (segment == 0 || segment[0] == ROUTE_END ||
        nodesr.nowNode.nodenum >= MAP_NODE_COUNT)
    {
        return map_splice_fail(ROUTE_BUILD_INVALID_ARG);
    }

    current = nodesr.nowNode.nodenum;
    while (length < ROUTE_CAPACITY && segment[length] != ROUTE_END)
    {
        if (segment[length] >= MAP_NODE_COUNT)
            return map_splice_fail(ROUTE_BUILD_INVALID_NODE);
        if (getNextConnectNode(current, segment[length]) == MAP_NODE_INDEX_INVALID)
            return map_splice_fail(ROUTE_BUILD_DISCONNECTED);
        current = segment[length];
        length++;
    }

    if (length == 0u)
        return map_splice_fail(ROUTE_BUILD_INVALID_ARG);
    if (length >= ROUTE_CAPACITY)
        return map_splice_fail(ROUTE_BUILD_MALFORMED);

    /*
     * map.point already points after nodesr.nextNode. Replacing from
     * map.point - 1 swaps the preloaded next node as well as the remaining tail.
     */
    splice_index = (map.point == 0u) ? 0u : (uint8_t)(map.point - 1u);
    if ((uint16_t)splice_index + (uint16_t)length >= ROUTE_CAPACITY)
        return map_splice_fail(ROUTE_BUILD_FULL);

    for (i = 0u; i < length; i++)
        route[splice_index + i] = segment[i];
    route[splice_index + length] = ROUTE_END;
    for (i = (uint8_t)(splice_index + length + 1u); i < ROUTE_CAPACITY; i++)
        route[i] = ROUTE_END;

    next_index = getNextConnectNode(nodesr.nowNode.nodenum, route[splice_index]);
    if (next_index == MAP_NODE_INDEX_INVALID)
        return map_splice_fail(ROUTE_BUILD_DISCONNECTED);

    nodesr.nextNode = Node[next_index];
    map.point = (uint8_t)(splice_index + 1u);
    route_last_segment = 0u;
    route_phase_reset();
    return ROUTE_BUILD_OK;
}

/*
 * black 同层换门：在当前位置插入 detour 换门段落，段落末节点等于 reentry_node，
 * reentry_node 之后保留原主路线的后半段（即从 reentry_node 的下一个节点继续）。
 * detour 例：{N4,N5,D3,N8}，reentry_node=N8。
 */
RouteBuildStatus_t Map_SpliceInsertDetour(const uint8_t *detour,
                                          uint8_t reentry_node)
{
    uint8_t tail[ROUTE_CAPACITY];
    uint8_t current;
    uint8_t splice_index;
    uint8_t detour_len = 0u;
    uint8_t tail_len = 0u;
    uint8_t next_index;
    uint8_t total;
    uint8_t i;

    if (detour == 0 || detour[0] == ROUTE_END ||
        nodesr.nowNode.nodenum >= MAP_NODE_COUNT)
        return map_splice_fail(ROUTE_BUILD_INVALID_ARG);

    current = nodesr.nowNode.nodenum;
    while (detour_len < ROUTE_CAPACITY && detour[detour_len] != ROUTE_END)
    {
        if (detour[detour_len] >= MAP_NODE_COUNT)
            return map_splice_fail(ROUTE_BUILD_INVALID_NODE);
        if (getNextConnectNode(current, detour[detour_len]) == MAP_NODE_INDEX_INVALID)
            return map_splice_fail(ROUTE_BUILD_DISCONNECTED);
        current = detour[detour_len];
        detour_len++;
    }
    if (detour_len == 0u || detour[detour_len - 1u] != reentry_node)
        return map_splice_fail(ROUTE_BUILD_INVALID_ARG);
    if (detour_len >= ROUTE_CAPACITY)
        return map_splice_fail(ROUTE_BUILD_MALFORMED);

    /* 原路线在 reentry_node 之后的后半段 */
    splice_index = (map.point == 0u) ? 0u : (uint8_t)(map.point - 1u);
    i = 0u;
    while ((uint16_t)splice_index + (uint16_t)i < ROUTE_CAPACITY &&
           route[splice_index + i] != ROUTE_END &&
           route[splice_index + i] != reentry_node)
        i++;
    if ((uint16_t)splice_index + (uint16_t)i >= ROUTE_CAPACITY ||
        route[splice_index + i] != reentry_node)
        return map_splice_fail(ROUTE_BUILD_DISCONNECTED);

    /* 从 reentry_node 之后复制原其余路段 */
    i++; /* 跳过 reentry_node */
    while (tail_len < ROUTE_CAPACITY &&
           (uint16_t)splice_index + (uint16_t)i < ROUTE_CAPACITY)
    {
        if (route[splice_index + i] == ROUTE_END)
            break;
        tail[tail_len++] = route[splice_index + i];
        i++;
    }

    total = (uint8_t)(detour_len + tail_len);
    if ((uint16_t)splice_index + (uint16_t)total >= ROUTE_CAPACITY)
        return map_splice_fail(ROUTE_BUILD_FULL);

    for (i = 0u; i < detour_len; i++)
        route[splice_index + i] = detour[i];
    for (i = 0u; i < tail_len; i++)
        route[splice_index + detour_len + i] = tail[i];
    route[splice_index + total] = ROUTE_END;
    for (i = (uint8_t)(splice_index + total + 1u); i < ROUTE_CAPACITY; i++)
        route[i] = ROUTE_END;

    next_index = getNextConnectNode(nodesr.nowNode.nodenum, detour[0]);
    if (next_index == MAP_NODE_INDEX_INVALID)
        return map_splice_fail(ROUTE_BUILD_DISCONNECTED);
    nodesr.nextNode = Node[next_index];
    map.point = (uint8_t)(splice_index + 1u);
    route_last_segment = 0u;
    route_phase_reset();
    return ROUTE_BUILD_OK;
}

static void cross_line_init(void)
{
    Chassis_ClearMileage();
    arrival_detector_reset();
    temp_track_reset(nodesr.nowNode.flag);
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
    else
        return;

    /*
     * 已在50%处主动切换循线模式，标记 FINAL，
     * 避免到达检测器在70%时再次做 TEMP_TRACK_PRIMARY 切换导致不必要的10cm清出。
     */
    temp_track_phase = TEMP_TRACK_FINAL;
    route_state = 3;
}

static void cross_arrive_slowdown(void);

static float get_detect_ratio(void)
{
    /* N16→N18(25cm) 用0.85，其余用0.7 */
    if (nodesr.nowNode.nodenum == N18 && nodesr.nowNode.step == 25)
        return 0.85f;
    return ROUTE_DETECT_RATIO;
}

static void cross_detect_start(void)
{
    if (!detect_started &&
        fabsf(Chassis_GetMileage()) >= get_detect_ratio() * nodesr.nowNode.step)
    {
        detect_started = 1;
        cross_arrive_slowdown();
    }
}

static void cross_arrive_slowdown(void)
{
    float ad;
    float ad2;

    if (fabsf(Chassis_GetMileage()) < ROUTE_SLOW_RATIO * nodesr.nowNode.step)
        return;

    ad  = fabsf(need2turn(getAngleZ(), nodesr.nextNode.angle));
    ad2 = fabsf(need2turn(nodesr.nowNode.angle, nodesr.nextNode.angle));

    if (route_need_turn(ad, ad2))
    {
        float turn_speed;

        if (nodesr.nowNode.nodenum == N19)
        {
            turn_speed = SPEED0;
        }
        else if (nodesr.nowNode.function == UpStage)
        {
            /* 普通平台(P1/P3/P4/P5)接近段允许稍快：25 → 30 */
            turn_speed = SPEED2;
        }
        else
        {
            turn_speed = SPEED1;
        }
        Chassis_SetTargetSpeed(turn_speed);
    }
    else if (nodesr.nextNode.speed < nodesr.nowNode.speed)
    {
        Chassis_SetTargetSpeed(nodesr.nextNode.speed);
    }
}

static void cross_arrive_check(void)
{
    /*
     * 检查优先级（从高到低）：
     *   1. 无条件硬保护 — 重入保护（不依赖检测窗口，永远生效）
     *   2. 检测窗口门控 — detect_started（70%里程后才开放检测）
     *   3. 辅助逻辑     — 临时巡线清出
     *   4. 到达检测     — 传感器模式匹配
     *
     * 设计意图：detect_started 屏蔽前 70% 段长，统一替代逐段盲区。
     * 硬保护必须排在检测窗口之前。如果 detect_started 因任何原因
     * （里程累积误差、编码器竞争等）被提前置 1，重入保护仍能拦截。
     */

    /* ============ 第一层：无条件硬保护（优先级最高） ============ */

    /* 节点重入保护：切换节点后必须走够保护距离才允许检测 */
    if (fabsf(Chassis_GetMileage() - node_entry_mileage) < NODE_REENTRY_CM)
        return;

    /* ============ 第二层：检测窗口门控 ============ */

    /* 检测窗口未到（前70%里程不检测）或已到达 */
    if (!detect_started || route_arrived())
        return;

    /* ============ 第三层：辅助逻辑 ============ */

    /* 临时巡线清出未完成 */
    if (!temp_track_clearance_done())
        return;

    /* ============ 第四层：到达检测 ============ */

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
        cross_arrive_slowdown();
    }

    /* 里程超标强制到达：走超段长120%仍未检测到节点，强制触发 */
    /* N8→N12例外：MUL2MUL检测可靠，不启用兜底 */
    /* N16→N18例外：N16与N18共享DRIGHT图案，靠检测到达 */
    /* B5→N19例外：DRIGHT|CRIGHT双检测可靠 */
    /* P3→N3例外：DRIGHT检测可能在N3路口漏检，205cm段走完即强制到达 */
    if (nodesr.nowNode.nodenum == N3 && nodesr.nowNode.step == 205
        && fabsf(Chassis_GetMileage()) >= 205.0f)
    {
        route_set_arrived();
        cross_arrive_slowdown();
    }
    else if (!(nodesr.nowNode.nodenum == N12 && nodesr.nowNode.step == 270)
        && !(nodesr.nowNode.nodenum == N18 && nodesr.nowNode.step == 25)
        && !(nodesr.nowNode.nodenum == N19 && nodesr.nowNode.step == 100)
        && fabsf(Chassis_GetMileage()) >= nodesr.nowNode.step * ROUTE_FORCE_RATIO)
    {
        route_set_arrived();
        cross_arrive_slowdown();
    }
}

static void cross_line_update(void)
{
    if (route_state == 0)
        cross_line_init();

    if (route_state == 1)
        cross_line_start();

    cross_track_switch();
    cross_detect_start();
    cross_arrive_check();

    /*
     * 主动岔口防护：仅保留Go_Line层面的误差融合（scaner.c），
     * 不再额外降速，避免与cross_arrive_slowdown叠加造成巡线不稳。
     */

    cross_arrive_slowdown();

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
    float drive_cm = (nodesr.nowNode.nodenum == N20) ? 0.0f :
                     (nodesr.nowNode.nodenum == N18) ? 15.0f : 18.0f;
    float lock_angle = (nodesr.nowNode.nodenum == N19) ? getAngleZ() : nodesr.nowNode.angle;

    /*
     * P3→N3→D4 专用：N3 被车头循迹板提前检测到，默认18cm不足以让
     * 车体转向中心进入路口。先沿P3→N3方向进入25cm，转向后再沿D4
     * 方向锁头前进8cm，确保循迹板真正落到D4方向线路上再交还Cross。
     */
    if (nodesr.lastNode.nodenum == P3 &&
        nodesr.nowNode.nodenum == N3 &&
        nodesr.nextNode.nodenum == D4)
    {
        float turn_amt;
        float compensated;

        Chassis_DriveDistance_Blocking(
            is_Gyro,
            P3_N3_TURN_FORWARD_CM,
            SPEED1,
            nodesr.nowNode.angle);
        CarBrake();
        vTaskDelay(DELAY_SHORT);

        turn_amt = need2turn(nodesr.nowNode.angle, nodesr.nextNode.angle);
        compensated = nodesr.nowNode.angle + turn_amt * TURN_SCALE;
        while (compensated > 180.0f)
            compensated -= 360.0f;
        while (compensated <= -180.0f)
            compensated += 360.0f;

        Chassis_Turn_By_StopGyro_Blocking(compensated, getAngleZ());
        CarBrake();
        vTaskDelay(DELAY_SHORT);

        Chassis_DriveDistance_Blocking(
            is_Gyro,
            P3_N3_POST_TURN_FORWARD_CM,
            SPEED1,
            nodesr.nextNode.angle);
        return;
    }

    /* N18 三岔路口：转弯前先沿来路前进18cm驶离横线，再原地转90°，
       最后朝B5方向锁头前进15cm驶入目标线正上方，避免转弯后传感器
       同时看到去B5的线与去N16的岔路，被scan_right_line锁错带偏。 */
    if (nodesr.nowNode.nodenum == N18)
    {
        float turn_amt;
        float compensated;

        Chassis_DriveDistance_Blocking(is_Gyro, 18.0f, SPEED1, nodesr.nowNode.angle);
        CarBrake();
        vTaskDelay(DELAY_SHORT);

        turn_amt = need2turn(nodesr.nowNode.angle, nodesr.nextNode.angle);
        compensated = nodesr.nowNode.angle + turn_amt * TURN_SCALE;
        while (compensated > 180.0f)  compensated -= 360.0f;
        while (compensated <= -180.0f) compensated += 360.0f;

        Chassis_Turn_By_StopGyro_Blocking(compensated, getAngleZ());
        CarBrake();
        vTaskDelay(DELAY_SHORT);

        Chassis_DriveDistance_Blocking(is_Gyro, 15.0f, SPEED1, nodesr.nextNode.angle);
        return;
    }

    Chassis_DriveDistance_Blocking(is_Gyro, drive_cm, SPEED1, lock_angle);
    CarBrake();
    vTaskDelay(DELAY_SHORT);
    {
        uint8_t n19_turn = (nodesr.nowNode.nodenum == N19);
        float turn_amt = need2turn(nodesr.nowNode.angle, nodesr.nextNode.angle);
        float compensated = nodesr.nowNode.angle + turn_amt * TURN_SCALE;
        while (compensated > 180.0f)  compensated -= 360.0f;
        while (compensated <= -180.0f) compensated += 360.0f;

        /* N19原地90°转时暂时关闭保护，避免车身抖动误触发 */
        if (n19_turn) {
            Chassis_DisableRollProtection();
            Chassis_DisableYawJumpProtection();
        }
        Chassis_Turn_By_StopGyro_Blocking(compensated, getAngleZ());
        if (n19_turn) {
            Chassis_EnableRollProtection();
            Chassis_EnableYawJumpProtection();
        }
    }

}

static void cross_run_turn(void)
{

    float run_drive_cm = 15.0f;
    Chassis_DriveDistance_Blocking(is_Gyro, run_drive_cm, SPEED1, getAngleZ());

    /* 出节点后停车转弯，复用平台稳定转向逻辑。 */

    float turn_amt_run = need2turn(nodesr.nowNode.angle, nodesr.nextNode.angle);
    float compensated_run = nodesr.nowNode.angle + turn_amt_run * TURN_SCALE;
    while (compensated_run > 180.0f)  compensated_run -= 360.0f;
    while (compensated_run <= -180.0f) compensated_run += 360.0f;

    CarBrake();
    vTaskDelay(DELAY_SHORT);
    Chassis_Turn_By_StopGyro_Blocking(compensated_run, getAngleZ());
    return;

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
    Chassis_DriveDistance_Blocking(is_Gyro, N2_B1_PASS_CM, SPEED1, nodesr.nowNode.angle);
    LEFT_RIGHT_LINE = CENTER_LINE_MODE;
}

static uint8_t cross_route_end(void)
{
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
    HmiDisplay_RecordArrival(nodesr.nowNode.nodenum, nodesr.nowNode.function);

    /* 已越过终点：上一段即最后一段，停车结束本轮 */
    if (route_last_segment)
    {
        cross_route_end();
        return;
    }

    /* 推进到最后一段（其后是 ROUTE_END）：不计算 nextNode，
       标记后先走完本段（含 UpStageP2 等障碍），再结束 */
    if (route[map.point] == ROUTE_END)
        route_last_segment = 1;
    else
        nodesr.nextNode = Node[getNextConnectNode(nodesr.nowNode.nodenum, route[map.point++])];

    cross_special_n2_b1();

    /* 返程 N3→N4：进入前校准一次航向，抵消前面 DOOR/岔路后的 yaw 漂移 */
    if (nodesr.lastNode.nodenum == N3 && nodesr.nowNode.nodenum == N4)
        mpuZreset(imu.yaw, nodesr.nowNode.angle);

    /* 校准点：RESTMPUZ 边进入时把绝对航向拉回地图帧，抑制长赛程 yaw 漂移 */
    if ((nodesr.nowNode.flag & RESTMPUZ) == RESTMPUZ)
        mpuZreset(imu.yaw, nodesr.nowNode.angle);

    Chassis_ClearMileage();
    node_entry_mileage = 0.0f;
    arrival_detector_reset();
    Chassis_SetTargetSpeed(nodesr.nowNode.speed);

    /* N20→P8：锁头直走，锁地图航向（避免岔路口拉偏） */
    if (nodesr.lastNode.nodenum == N20 && nodesr.nowNode.nodenum == P8)
    {
        Chassis_SetMode(is_Gyro);
        angle.AngleG = nodesr.nowNode.angle;
    }
    else
    {
        Chassis_SetMode(is_Line);
    }

    cross_line_protect_on();

    /* P7→C10、N20→P8：禁用丢线保护 */
    if ((nodesr.lastNode.nodenum == P7 && nodesr.nowNode.nodenum == C10) ||
        (nodesr.lastNode.nodenum == N20 && nodesr.nowNode.nodenum == P8))
    {
        cross_line_protect_off();
    }
}

static void cross_turn_update(void)
{
    float ad;
    float ad2;

    if (!route_arrived() || Chassis_IsStopLocked())
        return;

    /* P2→N2 到达后停车延时，稳定车身后再继续 N2→B1 过桥 */
    if (nodesr.nowNode.nodenum == P2 && nodesr.nextNode.nodenum == N2)
    {
        CarBrake();
        vTaskDelay(pdMS_TO_TICKS(P2_N2_STOP_MS));
    }

    /*
     * 有障碍物函数（UpStage/Bridge/Hill 等）的节点：
     * 先返回，让下一周期 cross_barrier_update() 执行障碍物函数。
     * 障碍物完成后 barrier_done() 置 NODE_ARRIVED_FLAG，
     * cross_barrier_update() 内部 route_phase_reset()，
     * 然后本函数再处理转弯。防止到达检测同周期内直接转弯跳过障碍物。
     */
    if (!(nodesr.nowNode.function == NONE || nodesr.nowNode.function == 0))
        return;

    cross_line_protect_off();

    ad  = fabsf(need2turn(getAngleZ(), nodesr.nextNode.angle));
    ad2 = fabsf(need2turn(nodesr.nowNode.angle, nodesr.nextNode.angle));

    /*
     * 转弯优先级：
     *   1. STOPTURN 或角度 > 90° → 停车原地转（不受 route_need_turn 约束）
     *   2. 无需转弯（角度差 < 10°）  → 直通
     *   3. 其余                      → 行进中转
     *
     * STOPTURN 必须独立判断：两个连续段角度相同时 route_need_turn 返回 false，
     * 若把 STOPTURN 放在 else 分支里会被跳过，导致节点不停车直接冲过去。
     */
    if ((nodesr.nowNode.flag & STOPTURN) == STOPTURN || ad >= TURN_STOP_ANGLE)
        cross_stop_turn();
    else if (!route_need_turn(ad, ad2))
    {
        cross_pass_turn();
    }
    else
        cross_run_turn();

    /*
     * barrier=NONE 的节点：无需障碍物处理，直接推进并重置阶段，
     * 下一周期进入新段的 cross_line_update()。
     *
     * 有 barrier 的节点（Hill/Door/Bridge等）：保留 is_near_end=1，
     * 不推进、不重置，留给下一周期 cross_barrier_update() 执行障碍物。
     * 障碍物完成后由 barrier_done() 置 NODE_ARRIVED_FLAG，
     * cross_barrier_update() 末尾的 route_phase_reset() 清理状态，
     * 然后 cross_turn_update() 再次执行推进。
     */
    if (nodesr.nowNode.function == NONE || nodesr.nowNode.function == 0)
    {
        route_clear_arrived();
        cross_node_advance();
        route_phase_reset();
    }
}

/**
 * @brief  Cross 状态机 - 节点间处理核心
 * @details 所有运动通过 chassis_api 控制（参考 xunbao 架构）。
 *          状态流程：
 *          1. 路径初始化 (route_state=0): 清零里程，设置巡线模式，使能巡线保护
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

    /* B6→N20到达后停车1秒 */
    if (nodesr.lastNode.nodenum == B6 && nodesr.nowNode.nodenum == N20 && route_arrived())
    {
        CarBrake();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    cross_turn_update();
}
