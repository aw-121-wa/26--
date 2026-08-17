#include "traffic_route.h"

#ifndef TRAFFIC_ROUTE_UNIT_TEST
#include "map.h"
#include "route_catalog.h"
#include "../vision/vision_api.h"
#include "../chassis/chassis_api.h"
#include "../../Task/motor_task.h"
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
 * 8 个方向全部纳入：4 个正向 + 4 个返程反向。返回 0..3=门对号，0xFF=非门边。 */
static uint8_t current_gate(uint8_t *dir)
{
    if (dir == 0)
        return 0xFFu;

    *dir = (uint8_t)GATE_DIR_FORWARD;
    if (nodesr.lastNode.nodenum == N5 && nodesr.nowNode.nodenum == N12)
        return 0u;
    if (nodesr.lastNode.nodenum == N5 && nodesr.nowNode.nodenum == N8)
        return 1u;
    if (nodesr.lastNode.nodenum == N3 && nodesr.nowNode.nodenum == N8)
        return 2u;
    if (nodesr.lastNode.nodenum == N3 && nodesr.nowNode.nodenum == N10)
        return 3u;

    *dir = (uint8_t)GATE_DIR_RETURN;
    if (nodesr.lastNode.nodenum == N12 && nodesr.nowNode.nodenum == N5)
        return 0u;
    if (nodesr.lastNode.nodenum == N8 && nodesr.nowNode.nodenum == N5)
        return 1u;
    if (nodesr.lastNode.nodenum == N8 && nodesr.nowNode.nodenum == N3)
        return 2u;
    if (nodesr.lastNode.nodenum == N10 && nodesr.nowNode.nodenum == N3)
        return 3u;

    return 0xFFu;
}

/* 根据门对所在层选定“进入次序”(step)：外层门[0/3]首次进入，内层门[1/2]二次进入。
 * 该次序是门对物理属性，与来向无关；仅正向可通行拼接时使用。 */
static TrafficRouteStep_t select_step(uint8_t gate_index)
{
    if (gate_index == 1u || gate_index == 2u)
        return TRAFFIC_ROUTE_STEP_SECOND_PASSABLE;
    return TRAFFIC_ROUTE_STEP_FIRST_PASSABLE;
}

/* 黑门换线：退回门源侧，改走同层对面那扇门重新进入，并保留原路线重入门后的后半段。
 * 不做“只换一次”限制、不看对侧缓存颜色：换过去若仍为黑，到达该门时会重新识别并再次换线；
 * 同层双黑就在两扇门之间反复切换，直到读到 GREEN/BLUE 或视觉失败。 */
static TrafficRouteStatus_t black_swap(uint8_t gate_index, uint8_t dir)
{
    /* 正向黑门换线：经 N4 绕行同层另一扇门。 */
    static const uint8_t swap_inner_a[] = {N4, N5, N8};   /* N3→N8 黑 → 经 N4 改走 N5→N8 */
    static const uint8_t swap_inner_b[] = {N4, N3, N8};   /* N5→N8 黑 → 经 N4 改走 N3→N8 */
    static const uint8_t swap_outer_a[] = {N4, N5, N12};  /* N3→N10 黑 → 经 N4 改走 N5→N12 */
    static const uint8_t swap_outer_b[] = {N4, N3, N10};  /* N5→N12 黑 → 经 N4 改走 N3→N10 */

    /* 返程黑门换线（用户指定路径，reentry=N4，收敛回返程尾段）：
     * gate3→gate0: N10→N12→N5→N4；gate0→gate3: N12→N8→N10→N3→N4；
     * gate2→gate1: N8→N5→N4；        gate1→gate2: N8→N3→N4。 */
    static const uint8_t ret_swap_g3_g0[] = {N12, N5, N4};
    static const uint8_t ret_swap_g0_g3[] = {N8, N10, N3, N4};
    static const uint8_t ret_swap_g2_g1[] = {N5, N4};
    static const uint8_t ret_swap_g1_g2[] = {N3, N4};

    const uint8_t *detour = 0;
    uint8_t reentry = 0u;
    uint8_t src = 0u;
    RouteBuildStatus_t status;

    if (dir == GATE_DIR_FORWARD)
    {
        switch (gate_index)
        {
        case 0u: detour = swap_outer_b; reentry = N10; src = N3; break; /* N5→N12 黑 → 换 N3→N10 */
        case 1u: detour = swap_inner_b; reentry = N8;  src = N3; break; /* N5→N8  黑 → 换 N3→N8  */
        case 2u: detour = swap_inner_a; reentry = N8;  src = N5; break; /* N3→N8  黑 → 换 N5→N8  */
        case 3u: detour = swap_outer_a; reentry = N12; src = N5; break; /* N3→N10 黑 → 换 N5→N12 */
        default:
            return TRAFFIC_ROUTE_STATUS_NO_ROUTE;
        }
    }
    else
    {
        switch (gate_index)
        {
        case 0u: detour = ret_swap_g0_g3; reentry = N4; src = N12; break; /* N12→N5 黑 → 换门3返 */
        case 1u: detour = ret_swap_g1_g2; reentry = N4; src = N8;  break; /* N8→N5  黑 → 换门2返 */
        case 2u: detour = ret_swap_g2_g1; reentry = N4; src = N8;  break; /* N8→N3  黑 → 换门1返 */
        case 3u: detour = ret_swap_g3_g0; reentry = N4; src = N10; break; /* N10→N3 黑 → 换门0返 */
        default:
            return TRAFFIC_ROUTE_STATUS_NO_ROUTE;
        }
    }

    /* 退回门源侧：锁来路航向(当前门边角度)直退 40cm（返程同样锁反向边角度）。 */
    Chassis_ClearMileage();
    Chassis_DriveDistance_Blocking(is_Gyro, 40.0f, -25.0f, nodesr.nowNode.angle);
    CarBrake();
    vTaskDelay(pdMS_TO_TICKS(80));

    nodesr.nowNode.nodenum = src;
    status = Map_SpliceInsertDetour(detour, reentry);
    if (status != ROUTE_BUILD_OK)
        return TRAFFIC_ROUTE_STATUS_SPLICE_FAILED;

    /* 倒车后原地转向 detour 首段方向，并把地图角度同步为实车 yaw，
     * 使后续 need2turn≈0 直通巡线。同时清掉原门边残留的 nowNode.flag：
     * 尤其返程边 N10→N3 带 STOPTURN，若不清残留，换门完成后 cross_stop_turn
     * 会再前进约 18cm 并做一次无效转向。 */
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

        status = Map_SpliceRemainingRoute(segment);
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