#include "traffic_route.h"

#ifndef TRAFFIC_ROUTE_UNIT_TEST
#include <math.h>
#include "map.h"
#include "route_catalog.h"
#include "imu.h"
#include "../vision/vision_api.h"
#include "../chassis/chassis_api.h"
#include "../../Task/motor_task.h"
#include "scaner.h"
#include "FreeRTOS.h"
#include "task.h"
#endif

#define TRAFFIC_ROUTE_NO_ROUTE 0u

/* 临时开关：置1禁用视觉红绿灯（跳过扫描与动态改线，纯跑原路线）；置0恢复。 */
#define TRAFFIC_ROUTE_VISION_DISABLED 0

/* 最近一次扫描识别的颜色：仅用于 HMI 显示。 */
static uint8_t last_effective_color = TRAFFIC_ROUTE_COLOR_NONE;

#ifndef TRAFFIC_ROUTE_UNIT_TEST
/* 扫描方向启发式（实车标定规则）：第一次实际通过门之前，所有门都朝右扫；
 * 第一次通过门之后，所有门朝左扫。与门对/来向无关的全局行经方向规则。 */
static uint8_t gate_first_passed = 0u;

/* 连续黑门换门计数防护（按 门对×来向 分槽）：同层双黑会反复换门，加次数上限
 * 避免死循环导致 splice 累积失败 + ForceStop 永久停锁。达到上限后按
 * "NONE 直接通过" 放行。每槽在一次换路链内累计；换路成功/正常通行后复位。 */
#define GATE_SWAP_MAX 3u
static uint8_t gate_swap_count[TRAFFIC_ROUTE_GATE_COUNT][2u] = {{0u, 0u}, {0u, 0u}, {0u, 0u}, {0u, 0u}};

static void gate_swap_reset(uint8_t gate_index, uint8_t dir)
{
    if (gate_index < TRAFFIC_ROUTE_GATE_COUNT && dir <= (uint8_t)GATE_DIR_RETURN)
        gate_swap_count[gate_index][dir] = 0u;
}
#endif

TrafficRouteColor_t TrafficRoute_NormalizeVisionColor(uint8_t vision_value)
{
    switch (vision_value)
    {
    case 1u:
        return TRAFFIC_ROUTE_COLOR_GREEN;
    case 2u:
        return TRAFFIC_ROUTE_COLOR_BLUE;
    case 3u:
        return TRAFFIC_ROUTE_COLOR_BLACK;
    default:
        return TRAFFIC_ROUTE_COLOR_NONE;
    }
}

/* 通行规则（固定）：GREEN 单向通行、BLUE 双向可通行——两者在任一来向都可过；
 * BLACK 禁止；NONE/识别失败由上层决定（现为直接通过）。
 * is_return_trip 保留参数仅为 API 兼容，蓝色不再区分来向。 */
uint8_t TrafficRoute_IsColorPassable(TrafficRouteColor_t color,
                                     uint8_t is_return_trip)
{
    (void)is_return_trip;
    if (color == TRAFFIC_ROUTE_COLOR_GREEN ||
        color == TRAFFIC_ROUTE_COLOR_BLUE)
        return 1u;
    return 0u;
}

static int8_t clue_group(uint8_t clue_a, uint8_t clue_b)
{
    if (clue_a == 5u && clue_b == 7u) return 0;
    if (clue_a == 5u && clue_b == 8u) return 1;
    if (clue_a == 6u && clue_b == 7u) return 2;
    if (clue_a == 6u && clue_b == 8u) return 3;
    return -1;
}

uint8_t TrafficRoute_SelectDoorRouteNumber(uint8_t clue_a, uint8_t clue_b,
                                           TrafficRouteStep_t step,
                                           TrafficRouteColor_t color)
{
    int8_t group = clue_group(clue_a, clue_b);

    if (group < 0)
        return TRAFFIC_ROUTE_NO_ROUTE;

    switch (step)
    {
    case TRAFFIC_ROUTE_STEP_FIRST_PASSABLE:
        if (!TrafficRoute_IsColorPassable(color, 0u))
            return TRAFFIC_ROUTE_NO_ROUTE;
        return (uint8_t[]){1u, 2u, 5u, 6u}[(uint8_t)group];

    case TRAFFIC_ROUTE_STEP_SECOND_PASSABLE:
        if (!TrafficRoute_IsColorPassable(color, 0u))
            return TRAFFIC_ROUTE_NO_ROUTE;
        return (uint8_t[]){3u, 4u, 7u, 8u}[(uint8_t)group];

    case TRAFFIC_ROUTE_STEP_OUTER_FALLBACK:
        if (color != TRAFFIC_ROUTE_COLOR_BLACK)
            return TRAFFIC_ROUTE_NO_ROUTE;
        return clue_a == 5u ? 9u : 10u;

    case TRAFFIC_ROUTE_STEP_LATE_BLOCK:
        if (color != TRAFFIC_ROUTE_COLOR_BLACK)
            return TRAFFIC_ROUTE_NO_ROUTE;
        return (uint8_t)(11u + (uint8_t)group);

    default:
        return TRAFFIC_ROUTE_NO_ROUTE;
    }
}

uint8_t TrafficRoute_GetLastColor(void)
{
    return last_effective_color;
}

#ifndef TRAFFIC_ROUTE_UNIT_TEST
/* 识别当前过门边属于哪个门对，并同时输出来向(FORWARD/RETURN)。
 * 门已实体化为 D2~D5 独立节点：nowNode 即门节点；方向由 lastNode(来向端点)判断。
 * FORWARD=从内层端点(N5/N3)进入，RETURN=从外层端点(N12/N8/N10)进入。
 * 返回 0..3=门对号，0xFF=非门边。 */
static uint8_t current_gate(uint8_t *dir)
{
    if (dir == 0)
        return 0xFFu;

    switch (nodesr.nowNode.nodenum)
    {
    case D2:   /* 门2：N5 ↔ N12 */
        *dir = (nodesr.lastNode.nodenum == N5) ? (uint8_t)GATE_DIR_FORWARD : (uint8_t)GATE_DIR_RETURN;
        return 0u;
    case D3:   /* 门3：N5 ↔ N8 */
        *dir = (nodesr.lastNode.nodenum == N5) ? (uint8_t)GATE_DIR_FORWARD : (uint8_t)GATE_DIR_RETURN;
        return 1u;
    case D4:   /* 门4：N3 ↔ N8 */
        *dir = (nodesr.lastNode.nodenum == N3) ? (uint8_t)GATE_DIR_FORWARD : (uint8_t)GATE_DIR_RETURN;
        return 2u;
    case D5:   /* 门5：N3 ↔ N10 */
        *dir = (nodesr.lastNode.nodenum == N3) ? (uint8_t)GATE_DIR_FORWARD : (uint8_t)GATE_DIR_RETURN;
        return 3u;
    default:
        return 0xFFu;
    }
}

/* 根据门对所在层选定“进入次序”(step)：外层门[0/3]首次进入，内层门[1/2]二次进入。
 * 该次序是门对物理属性，与来向无关；仅正向可通行拼接时使用。 */
static TrafficRouteStep_t select_step(uint8_t gate_index)
{
    if (gate_index == 1u || gate_index == 2u)
        return TRAFFIC_ROUTE_STEP_SECOND_PASSABLE;
    return TRAFFIC_ROUTE_STEP_FIRST_PASSABLE;
}

/* 黑门换线：退回来路源节点，改走同层对面那扇门重新进入，并保留原路线重入门后的后半段。
 * 不做“只换一次”限制、不看对侧缓存颜色：换过去若仍为黑，到达该门时会重新识别并再次换线；
 * 同层双黑就在两扇门之间反复切换，直到读到 GREEN/BLUE 或视觉失败。
 * 门已独立成节点：倒退距离用 nowNode.step（源节点→门的实测距离），源节点即 lastNode。 */
/* 黑门短倒车比例：带 BLACK_REVERSE_SHORT 标志的门（当前仅 N3→D4）按此比例折算
 * 倒车距离（actual_forward_cm × 0.30），补偿里程累计偏大。其余门原样倒回。
 * 该值为当前实车标定值；长期应查明为何 actual_forward_cm 比门边实际长度大约 3 倍。 */
#define BLACK_REVERSE_SHORT_SCALE  0.30f

static float black_reverse_distance(float actual_cm, u32 flag)
{
    if ((flag & BLACK_REVERSE_SHORT) != 0u)
        return actual_cm * BLACK_REVERSE_SHORT_SCALE;
    return actual_cm;
}

/* 黑门倒车传感检测：复用现有节点到达规则（MUL2MUL 三线 / MORELED 多灯），
 * 不新增标志位、不改地图边 flag。 */
#define BLACK_REVERSE_CONFIRM_CYCLES 3u
#define BLACK_REVERSE_TIMEOUT_MS     2000u
#define BLACK_REVERSE_MAX_CM         150.0f
#define BLACK_REVERSE_PERIOD_MS      5u

static uint8_t black_reverse_detect(u32 detect_flag)
{
    getline_error();
    if ((detect_flag & MUL2MUL) == MUL2MUL && Scaner.lineNum >= 3u)
        return 1u;
    if ((detect_flag & MORELED) == MORELED && Scaner.ledNum >= 5u)
        return 1u;
    return 0u;
}

/* Gyro 锁航向倒车，每周期主动刷新 getline_error()，按 detect_flag 规则检测回到源节点
 * （D4 正向黑门即回到 N3 多线区），连续 CONFIRM_CYCLES 个周期达标立即停车。
 * 带硬超时 / 里程上限 / 停锁保护。返回 1=已停靠，0=超时或失败。 */
static uint8_t black_reverse_until_flag(u32 detect_flag, float heading)
{
    TickType_t start = xTaskGetTickCount();
    uint8_t hits = 0u;

    Chassis_ClearMileage();
    Chassis_MotorControl(is_Gyro, -25.0f, -25.0f, heading);

    while (1)
    {
        if (black_reverse_detect(detect_flag))
        {
            if (++hits >= BLACK_REVERSE_CONFIRM_CYCLES)
            {
                CarBrake();
                return 1u;
            }
        }
        else
        {
            hits = 0u;
        }

        if (Chassis_IsStopLocked() ||
            (xTaskGetTickCount() - start) >= pdMS_TO_TICKS(BLACK_REVERSE_TIMEOUT_MS) ||
            fabsf(Chassis_GetMileage()) >= BLACK_REVERSE_MAX_CM)
        {
            CarBrake();
            return 0u;
        }
        vTaskDelay(pdMS_TO_TICKS(BLACK_REVERSE_PERIOD_MS));
    }
}

static TrafficRouteStatus_t black_swap(uint8_t gate_index, uint8_t dir)
{
    /* 正向黑门换线（含门节点）：经 N4 绕行同层另一扇门。 */
    static const uint8_t swap_inner_a[] = {N4, N5, D3, N8, ROUTE_END};   /* N3→N8  黑 → 经 N4 改走 N5→D3→N8 */
    static const uint8_t swap_inner_b[] = {N4, N3, D4, N8, ROUTE_END};   /* N5→N8  黑 → 经 N4 改走 N3→D4→N8 */
    static const uint8_t swap_outer_a[] = {N4, N5, D2, N12, ROUTE_END};  /* N3→N10 黑 → 经 N4 改走 N5→D2→N12 */
    static const uint8_t swap_outer_b[] = {N4, N3, D5, N10, ROUTE_END};  /* N5→N12 黑 → 经 N4 改走 N3→D5→N10 */

    /* 返程黑门换线（用户指定路径，reentry=N4，收敛回返程尾段）：
     * gate3→gate0: N10→N12→D2→N5→N4；gate0→gate3: N12→N8→N10→D5→N3→N4；
     * gate2→gate1: N8→D3→N5→N4；       gate1→gate2: N8→D4→N3→N4。 */
    static const uint8_t ret_swap_g3_g0[] = {N12, D2, N5, N4, ROUTE_END};
    static const uint8_t ret_swap_g0_g3[] = {N8, N10, D5, N3, N4, ROUTE_END};
    static const uint8_t ret_swap_g2_g1[] = {D3, N5, N4, ROUTE_END};
    static const uint8_t ret_swap_g1_g2[] = {D4, N3, N4, ROUTE_END};

    const uint8_t *detour = 0;
    uint8_t reentry = 0u;   /* detour 末节点，用于 Map_SpliceInsertDetour 校验 */
    uint8_t src = 0u;       /* 真实源节点（=lastNode，黑门退回点） */
    uint8_t node_before = 0u;
    float reverse_cm;
    float actual_forward_cm;
    RouteBuildStatus_t status;

    if (dir == GATE_DIR_FORWARD)
    {
        switch (gate_index)
        {
        case 0u: detour = swap_outer_b; reentry = N10; src = N5; break; /* N5→N12 黑 → 换 N3→D5→N10 */
        case 1u: detour = swap_inner_b; reentry = N8;  src = N5; break; /* N5→N8  黑 → 换 N3→D4→N8  */
        case 2u: detour = swap_inner_a; reentry = N8;  src = N3; break; /* N3→N8  黑 → 换 N5→D3→N8  */
        case 3u: detour = swap_outer_a; reentry = N12; src = N3; break; /* N3→N10 黑 → 换 N5→D2→N12 */
        default:
            return TRAFFIC_ROUTE_STATUS_NO_ROUTE;
        }
    }
    else
    {
        switch (gate_index)
        {
        case 0u: detour = ret_swap_g0_g3; reentry = N4; src = N12; break; /* N12→N5 黑 → 换门5返 */
        case 1u: detour = ret_swap_g1_g2; reentry = N4; src = N8;  break; /* N8→N5  黑 → 换门4返 */
        case 2u: detour = ret_swap_g2_g1; reentry = N4; src = N8;  break; /* N8→N3  黑 → 换门3返 */
        case 3u: detour = ret_swap_g3_g0; reentry = N4; src = N10; break; /* N10→N3 黑 → 换门2返 */
        default:
            return TRAFFIC_ROUTE_STATUS_NO_ROUTE;
        }
    }

    /* 换门次数上限防护（按 本门对×来向 分槽）：同层双黑反复换门 + splice 累积失败
     * 会死循环+永久停锁。达到上限改为直接通过（视同 NONE 处理），Barrier_Door 正常放行。 */
    if (gate_swap_count[gate_index][dir] >= GATE_SWAP_MAX)
    {
        return TRAFFIC_ROUTE_STATUS_NO_CHANGE;
    }

    /* 重定位来路航向：倒车前先 mpuZreset，把实车 yaw 校到地图帧，
     * 避免长赛程 yaw 漂移（可达 100°+）导致倒车/转向锁错来路航向。 */
    mpuZreset(imu.yaw, nodesr.nowNode.angle);

    /* 退回真实源节点：优先按“本轮实际前进里程”对称倒回。 */
    actual_forward_cm = fabsf(Chassis_GetMileage());

    if (gate_index == 2u && dir == GATE_DIR_FORWARD)
    {
        /* D4 正向黑门：复用 MUL2MUL(三线) 的节点检测规则做传感倒车，回到 N3 多线区停车，
         * 不再依赖里程比例。传感超时/失败则按实际进距对称倒回兜底。 */
        if (!black_reverse_until_flag(MUL2MUL, nodesr.nowNode.angle))
        {
            Chassis_ClearMileage();
            Chassis_DriveDistance_Blocking(is_Gyro, actual_forward_cm, -25.0f, nodesr.nowNode.angle);
            CarBrake();
            vTaskDelay(pdMS_TO_TICKS(80));
        }
    }
    else
    {
        reverse_cm = black_reverse_distance(actual_forward_cm, nodesr.nowNode.flag);
        Chassis_ClearMileage();
        Chassis_DriveDistance_Blocking(is_Gyro, reverse_cm, -25.0f, nodesr.nowNode.angle);
        CarBrake();
        vTaskDelay(pdMS_TO_TICKS(80));
    }

    /* 暂存原黑门节点，splice 失败降级放行时恢复，避免脏地图。 */
    node_before = nodesr.nowNode.nodenum;

    nodesr.nowNode.nodenum = src;
    status = Map_SpliceInsertDetour(detour, reentry);
    if (status != ROUTE_BUILD_OK)
    {
        /* 换路拼接失败（异常路线/嵌套换路时 reentry 缺失）：恢复 nowNode、
         * 计入一次失败，并按 reference 语义降级为“直接通过”放行——绝不 ForceStop
         * 永久停死。车走回原主路线由 Barrier_Door 正常推进。 */
        nodesr.nowNode.nodenum = node_before;
        if (gate_swap_count[gate_index][dir] < 0xFFu)
            gate_swap_count[gate_index][dir]++;
        return TRAFFIC_ROUTE_STATUS_NO_CHANGE;
    }
    gate_swap_reset(gate_index, dir);   /* 本次换路成功，复位该槽，允许后续正常再换 */

    /* 倒车后原地转向 detour 首段方向，并把地图角度同步为实车 yaw，
     * 使后续 need2turn≈0 直通巡线。同时清掉原门边残留的 nowNode.flag。 */
    CarBrake();
    {
        float target = nodesr.nextNode.angle;
        Chassis_Turn_By_StopGyro_Blocking(target, getAngleZ());
        nodesr.nowNode.angle = target;
    }
    nodesr.nowNode.flag = 0u;

    /* 黑门未通过，不改变扫描朝向（gate_first_passed 维持原值）。 */
    return TRAFFIC_ROUTE_STATUS_OK;
}

/* 正向可通行：把“门→远端”的真实边 prepend 到门路线前再拼接，
 * 保证车真实行驶 Dx→远端(40/45cm) 后才接门路线，不做逻辑瞬移。
 * 正向对端：D2→N12, D3→N8, D4→N8, D5→N10。 */
static RouteBuildStatus_t splice_forward_door_route(uint8_t gate_index,
                                                    const uint8_t *segment)
{
    static const uint8_t forward_far[TRAFFIC_ROUTE_GATE_COUNT] = {N12, N8, N8, N10};
    uint8_t combined[ROUTE_CAPACITY];
    uint8_t i = 0u;
    uint8_t j = 0u;

    if (gate_index >= TRAFFIC_ROUTE_GATE_COUNT || segment == 0)
        return ROUTE_BUILD_INVALID_ARG;

    combined[i++] = forward_far[gate_index];

    while (segment[j] != ROUTE_END)
    {
        if (i >= (uint8_t)(ROUTE_CAPACITY - 1u))
            return ROUTE_BUILD_FULL;
        combined[i++] = segment[j++];
    }
    combined[i] = ROUTE_END;

    return Map_SpliceRemainingRoute(combined);
}
#endif

#ifndef TRAFFIC_ROUTE_UNIT_TEST
TrafficRouteStatus_t TrafficRoute_HandleDoor(void)
{
#if TRAFFIC_ROUTE_VISION_DISABLED
    /* 视觉禁用：纯跑原路线，跳过红绿灯扫描与动态改线。 */
    return TRAFFIC_ROUTE_STATUS_NO_CHANGE;
#else
    VisionResult_t sample;
    VisionDirection_t direction;
    TrafficRouteColor_t effective;
    uint8_t gate_index;
    uint8_t dir = (uint8_t)GATE_DIR_FORWARD;
    TrafficRouteStep_t step;
    uint8_t route_number;
    const uint8_t *segment;
    RouteBuildStatus_t status;

    gate_index = current_gate(&dir);
    if (gate_index >= TRAFFIC_ROUTE_GATE_COUNT)
        return TRAFFIC_ROUTE_STATUS_NO_CHANGE;   /* 非门区边：直接放行 */

    /* 每次到达门都重新视觉识别：不缓存历史颜色、不因缓存跳过。
     * 扫描侧：第一次实际通过门之前所有门朝右，之后所有门朝左。 */
    direction = gate_first_passed ? VISION_DIRECTION_LEFT
                                  : VISION_DIRECTION_RIGHT;

    if (Vision_ScanSingleSide(direction, &sample) != VISION_STATUS_OK)
    {
        /* 识别失败/视觉通信超时：按规则直接通过当前门（允许继续）。
         * 车将实际穿过此门，此后扫描方向切换为朝左。 */
        last_effective_color = TRAFFIC_ROUTE_COLOR_NONE;
        gate_first_passed = 1u;
        return TRAFFIC_ROUTE_STATUS_SCAN_FAILED;
    }

    effective = TrafficRoute_NormalizeVisionColor(sample.value);
    last_effective_color = (uint8_t)effective;

    switch (effective)
    {
    case TRAFFIC_ROUTE_COLOR_GREEN:
    case TRAFFIC_ROUTE_COLOR_BLUE:
        if (dir == GATE_DIR_RETURN)
        {
            /* 返程可通行：直接保持当前 route 继续执行，绝不调用
             * RouteCatalog_GetDoor() / Map_SpliceRemainingRoute()。 */
            gate_first_passed = 1u;
            return TRAFFIC_ROUTE_STATUS_OK;
        }
        /* 正向可通行：按内外层选定门路线拼接。 */
        step = select_step(gate_index);
        route_number = TrafficRoute_SelectDoorRouteNumber(TRAFFIC_ROUTE_DEFAULT_CLUE_A,
                                                          TRAFFIC_ROUTE_DEFAULT_CLUE_B,
                                                          step,
                                                          effective);
        if (route_number == TRAFFIC_ROUTE_NO_ROUTE)
            return TRAFFIC_ROUTE_STATUS_NO_ROUTE;

        segment = RouteCatalog_GetDoor(route_number);
        if (segment == 0)
            return TRAFFIC_ROUTE_STATUS_NO_ROUTE;

        /* 正向可通行：不瞬移 nowNode，而是把“门→远端”真实边拼到门路线前，
         * 让车先真实走完 Dx→远端 再执行门路线（如 D3→N8→N12→...）。 */
        status = splice_forward_door_route(gate_index, segment);
        if (status != ROUTE_BUILD_OK)
            return TRAFFIC_ROUTE_STATUS_SPLICE_FAILED;

        gate_first_passed = 1u;
        return TRAFFIC_ROUTE_STATUS_OK;

    case TRAFFIC_ROUTE_COLOR_BLACK:
        /* 黑门：永远进入换门逻辑（无“只换一次”限制）。
         * 同层双黑时会在成对门之间反复切换，每次到达都重新识别。 */
        return black_swap(gate_index, dir);

    case TRAFFIC_ROUTE_COLOR_NONE:
    default:
        /* 视觉返回 NONE：按规则直接通过当前门，车将实际穿过，切换扫描朝向。 */
        gate_first_passed = 1u;
        return TRAFFIC_ROUTE_STATUS_OK;
    }
#endif
}
#else
TrafficRouteStatus_t TrafficRoute_HandleDoor(void)
{
    return TRAFFIC_ROUTE_STATUS_SCAN_FAILED;
}
#endif