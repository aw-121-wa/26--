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
#include "math.h"
#include "bsp_linefollower.h"

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
#define ARRIVE_CONFIRM_SAMPLES  3u
#define TEMP_TRACK_CLEAR_CM     10.0f
#define TURN_NEED_ANGLE         10.0f

/* ======================== 保护阈值 ======================== */

#define TURN_TIMEOUT_MS         500     /* 转弯硬超时 500ms */
#define TURN_TIMEOUT_CYCLES     (TURN_TIMEOUT_MS / CONTROL_CYCLE_MS)  /* 100 */
#define TURN_OSCILLATE_NEAR     8.0f    /* 接近目标阈值(度) */
#define TURN_OSCILLATE_FAR      20.0f   /* 震荡回弹阈值(度) */
#define NODE_REENTRY_CM         8.0f    /* 节点重入保护距离(cm) */
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

    /* 起点：P2平台 */
    nodesr.nowNode.nodenum = P2;
    nodesr.nowNode.angle = 0;
    nodesr.nowNode.function = NONE;
    nodesr.nowNode.speed = SPEED1;
    nodesr.nowNode.step = 10;
    nodesr.nowNode.flag = CLEFT | RIGHT_LINE;

    /* 获取第一个目标节点 */
    nodesr.nextNode = Node[getNextConnectNode(nodesr.nowNode.nodenum, route[map.point++])];
}

void mapInit1(void)
{
    map.point = 0;
    nodesr.flag = 0;

    nodesr.nowNode.nodenum = N2;
    nodesr.nowNode.angle = 0;
    nodesr.nowNode.function = NONE;
    nodesr.nowNode.speed = SPEED0;
    nodesr.nowNode.step = 2;
    nodesr.nowNode.flag = CLEFT | RIGHT_LINE;
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
    return 0;
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
        case BLBS:
            Barrier_WavedPlate(87.0f);
            break;
        case BLBL:
            Barrier_WavedPlate(160.0f);
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
    return route_state;
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

static void cross_line_protect_on(void)
{
    Chassis_EnableLineLostProtection();
}

static void cross_line_protect_off(void)
{
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
    if (!route_is_p2_to_n2())
        return;
    if (route_state != 2)
        return;
    if (fabsf(Chassis_GetMileage()) < ROUTE_HALF_RATIO * nodesr.nowNode.step)
        return;

    LEFT_RIGHT_LINE = RIGHT_LINE_MODE;
    route_state = 3;
}

static void cross_detect_start(void)
{
    if (!detect_started &&
        fabsf(Chassis_GetMileage()) >= ROUTE_DETECT_RATIO * nodesr.nowNode.step)
    {
        detect_started = 1;
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
        Chassis_SetTargetSpeed(SPEED1);
    }
    else if (nodesr.nextNode.speed < nodesr.nowNode.speed)
    {
        Chassis_SetTargetSpeed(nodesr.nextNode.speed);
    }
}

static void cross_arrive_check(void)
{
    if (!detect_started || route_arrived())
        return;

    if (!temp_track_clearance_done())
        return;

#if 0  /* 重入保护已不需要，下坡程序已重写，暂时关闭 */
    /* 节点重入保护：切换节点后必须走够保护距离才允许再次检测 */
    if (fabsf(Chassis_GetMileage() - node_entry_mileage) < NODE_REENTRY_CM)
        return;
#endif

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
        cross_arrive_slowdown();
    }

    /* 里程超标强制到达：走超步长20%没检测到就自动推进 */
    if (!route_arrived() && fabsf(Chassis_GetMileage()) >= nodesr.nowNode.step * 1.2f)
    {
        route_set_arrived();
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
    if (route_state == 0)
        cross_line_init();

    if (route_state == 1)
        cross_line_start();

    cross_track_switch();
    cross_detect_start();
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
    Chassis_DriveDistance_Blocking(is_Gyro, 15.0f, SPEED1, getAngleZ());
    CarBrake();
    vTaskDelay(DELAY_SHORT);
    Chassis_Turn_By_StopGyro_Blocking(nodesr.nextNode.angle, getAngleZ());
}

static void cross_run_turn(void)
{
    float err;
    uint16_t timeout;
    uint8_t  was_near;  /* 曾经接近过目标 */

    float old_speed_max;
    float old_kd;

    Chassis_DriveDistance_Blocking(is_Gyro, 5.0f, SPEED1, getAngleZ());

    /* 限幅差速 + 提高阻尼，防止暴力旋转和来回振荡 */
    old_speed_max = motor_all.GyroT_speedMax;
    old_kd = gyroT_pid_param.kd;
    motor_all.GyroT_speedMax = TURN_RUN_SPEED_MAX;
    gyroT_pid_param.kd = TURN_RUN_KD_BOOST;

    Chassis_SetMode(is_Turn);
    angle.AngleT = nodesr.nextNode.angle;

    err      = fabsf(need2turn(getAngleZ(), nodesr.nextNode.angle));
    was_near = 0;
    timeout  = TURN_TIMEOUT_CYCLES;  /* 500ms 硬超时 */

    while (err > TURN_DONE_DEADBAND)
    {
        vTaskDelay(CONTROL_CYCLE_MS);
        err = fabsf(need2turn(getAngleZ(), nodesr.nextNode.angle));

        /* 曾经接近目标(8°内)，现在又弹回超过20° → 震荡，立即刹车 */
        if (err < TURN_OSCILLATE_NEAR)
            was_near = 1;
        if (was_near && err > TURN_OSCILLATE_FAR)
        {
            CarBrake();
            break;
        }

        /* 硬超时兜底 */
        if (--timeout == 0)
        {
            CarBrake();
            break;
        }
    }

    motor_all.GyroT_speedMax = old_speed_max;
    gyroT_pid_param.kd = old_kd;
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

    if (cross_route_end())
        return;

    nodesr.nextNode = Node[getNextConnectNode(nodesr.nowNode.nodenum, route[map.point++])];
    cross_special_n2_b1();

    Chassis_ClearMileage();
    node_entry_mileage = 0.0f;
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
 *          2. 持续巡线 (route_state=1→2): 设速度，循线前进
 *          3. Temp模式切换: 首次确认岔口后切换，清出10cm再恢复到达检测
 *          4. P2专用切换 (route_state=2→3): 50%时由居中改为右循线
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
