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

/* 每个门已确定的红绿灯颜色：NONE=未扫描。 */
static uint8_t gate_color[TRAFFIC_ROUTE_GATE_COUNT] = {0u, 0u, 0u, 0u};
/* 本门是否已尝试过同层换门：黑门防护，防同层双黑在门对间无限横跳。 */
static uint8_t gate_swap_tried[TRAFFIC_ROUTE_GATE_COUNT] = {0u, 0u, 0u, 0u};
/* 是否已第一次实际通过某个门：为0时所有门朝右扫描，一旦第一次过了门就置1，之后所有门朝左。 */
static uint8_t first_gate_passed = 0u;
static uint8_t door_scan_count = 0u;
static uint8_t last_effective_color = TRAFFIC_ROUTE_COLOR_NONE;

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

uint8_t TrafficRoute_IsColorPassable(TrafficRouteColor_t color,
                                     uint8_t is_return_trip)
{
    if (color == TRAFFIC_ROUTE_COLOR_GREEN)
        return 1u;
    if (color == TRAFFIC_ROUTE_COLOR_BLUE)
        return is_return_trip ? 0u : 1u;
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
static uint8_t current_gate_index(void)
{
    if (nodesr.lastNode.nodenum == N5 && nodesr.nowNode.nodenum == N12)
        return 0u;
    if (nodesr.lastNode.nodenum == N5 && nodesr.nowNode.nodenum == N8)
        return 1u;
    if (nodesr.lastNode.nodenum == N3 && nodesr.nowNode.nodenum == N8)
        return 2u;
    if (nodesr.lastNode.nodenum == N3 && nodesr.nowNode.nodenum == N10)
        return 3u;
    return 0xFFu;
}

/* 根据门所在层选定“进入次序”(step)：外层门[0/3]首次进入，内层门[1/2]二次进入。
 * 进门次序仅用于非黑门选定默认路线；黑门已提前分流不走这里。 */
static TrafficRouteStep_t select_step(uint8_t gate_index)
{
    static const TrafficRouteStep_t kOuter = TRAFFIC_ROUTE_STEP_FIRST_PASSABLE;
    static const TrafficRouteStep_t kInner = TRAFFIC_ROUTE_STEP_SECOND_PASSABLE;

    if (gate_index == 1u || gate_index == 2u)
        return kInner;
    if (gate_index == 0u || gate_index == 3u)
        return kOuter;
    return (door_scan_count == 0u) ? kOuter : kInner;
}

/* 同层对面那扇门的下标：外层对(0,3)，内层对(1,2)。 */
static uint8_t opposite_gate(uint8_t gate_index)
{
    switch (gate_index)
    {
    case 0u: return 3u;
    case 1u: return 2u;
    case 2u: return 1u;
    case 3u: return 0u;
    default: return 0xFFu;
    }
}

TrafficRouteStatus_t TrafficRoute_HandleDoor(void)
{
#if TRAFFIC_ROUTE_VISION_DISABLED
    /* 视觉禁用：纯跑原路线，跳过红绿灯扫描与动态改线。 */
    return TRAFFIC_ROUTE_STATUS_NO_CHANGE;
#else
    TrafficRouteColor_t effective;
    TrafficRouteStep_t step;
    VisionResult_t sample;
    VisionDirection_t direction;
    uint8_t gate_index;
    uint8_t route_number;
    const uint8_t *segment;
    RouteBuildStatus_t status;

    gate_index = current_gate_index();
    /* 门区过渡边未枚举(索引非法)或门区边不在 4 门集合内：直接不动，走原主路线。 */
    if (gate_index >= TRAFFIC_ROUTE_GATE_COUNT)
        return TRAFFIC_ROUTE_STATUS_NO_CHANGE;

    /* 已存下且为可通行(非黑)信息：直接复用，不再动舵机扫描，也不再二次拼接。
     * 已存为黑的门仍需继续“再换一条”，故不在此处短路。 */
    if (gate_color[gate_index] != TRAFFIC_ROUTE_COLOR_NONE &&
        gate_color[gate_index] != TRAFFIC_ROUTE_COLOR_BLACK)
    {
        last_effective_color = gate_color[gate_index];
        return TRAFFIC_ROUTE_STATUS_NO_CHANGE;
    }

    /* 未存下信息：单边扫描。第一次实际通过门之前所有门朝右，
     * 第一次通过门之后所有门朝左。 */
    direction = first_gate_passed ? VISION_DIRECTION_LEFT
                                  : VISION_DIRECTION_RIGHT;
    if (Vision_ScanSingleSide(direction, &sample) != VISION_STATUS_OK)
        return TRAFFIC_ROUTE_STATUS_SCAN_FAILED;

    effective = TrafficRoute_NormalizeVisionColor(sample.value);
    last_effective_color = (uint8_t)effective;

    if (effective == TRAFFIC_ROUTE_COLOR_NONE)
        return TRAFFIC_ROUTE_STATUS_NO_CHANGE;

    gate_color[gate_index] = (uint8_t)effective; /* 存下来，后续复用 */

    /*
     * 黑色门：禁止通行。退回门源侧，改走同层对面的门重新进入，
     * 并保留原主路线在重入门(reentry)之后的后半段。
     * 若换过去的门仍为黑，其颜色已存为黑，再次经过时会再换下一条。
     */
    if (effective == TRAFFIC_ROUTE_COLOR_BLACK)
    {
        static const uint8_t swap_inner_a[] = {N4, N5, N8};   /* N3→N8 黑 → 经 N4 改走 N5→N8 */
        static const uint8_t swap_inner_b[] = {N4, N3, N8};   /* N5→N8 黑 → 经 N4 改走 N3→N8 */
        static const uint8_t swap_outer_a[] = {N4, N5, N12};  /* N3→N10 黑 → 经 N4 改走 N5→N12 */
        static const uint8_t swap_outer_b[] = {N4, N3, N10};  /* N5→N12 黑 → 经 N4 改走 N3→N10 */
        const uint8_t *detour;
        uint8_t reentry;
        uint8_t src;

        switch (gate_index)
        {
        case 0u: /* N5→N12 外层黑 → 换 N3→N10 */
            detour = swap_outer_b;
            reentry = N10;
            src = N3;
            break;
        case 1u: /* N5→N8 内层黑 → 换 N3→N8 */
            detour = swap_inner_b;
            reentry = N8;
            src = N3;
            break;
        case 2u: /* N3→N8 内层黑 → 换 N5→N8 */
            detour = swap_inner_a;
            reentry = N8;
            src = N5;
            break;
        case 3u: /* N3→N10 外层黑 → 换 N5→N12 */
            detour = swap_outer_a;
            reentry = N12;
            src = N5;
            break;
        default:
            return TRAFFIC_ROUTE_STATUS_NO_ROUTE;
        }

        /* 同层双黑防护(BH-1)：对面那扇门也存为黑，或本门已换过一次仍撞黑，
         * 换过去仍是死路——不再折腾换门，改走原主路线(闯黑)。 */
        if (gate_swap_tried[gate_index] ||
            gate_color[opposite_gate(gate_index)] == TRAFFIC_ROUTE_COLOR_BLACK)
        {
            return TRAFFIC_ROUTE_STATUS_NO_CHANGE;
        }
        gate_swap_tried[gate_index] = 1u;

        /* 退回门源侧：黑门处先倒车，随后从 src 方向重新进入。
         * 锁来路航向(nodesr.nowNode.angle)直退，保证沿 N3→N8 等门区中心线反向倒退，
         * 避免固定初值导致退成斜向。倒车距离已按实车标定为 40cm。 */
        {
            Chassis_ClearMileage();
            Chassis_DriveDistance_Blocking(is_Gyro, 40.0f, -25.0f, nodesr.nowNode.angle);
            CarBrake();
            vTaskDelay(pdMS_TO_TICKS(80));
        }
        nodesr.nowNode.nodenum = src;

        status = Map_SpliceInsertDetour(detour, reentry);
        if (status != ROUTE_BUILD_OK)
            return TRAFFIC_ROUTE_STATUS_SPLICE_FAILED;

        /*
         * 修复A/B：倒车后让车真正转向 detour 首段(进入另一扇门的方向)。
         * Map_SpliceInsertDetour 已把 nodesr.nextNode 设为 detour 首段边(src→detour[0])，
         * 用其绝对角作转向目标：从实车当前 yaw(getAngleZ) 原地转到该角，
         * 并把 nowNode.angle 同步为目标角，使后续 cross_stop_turn 的 need2turn≈0，
         * 不再用旧的固定假角/假位置二次强转，避免重新冲回门中心线。
         */
        CarBrake();
        {
            float target = nodesr.nextNode.angle;
            Chassis_Turn_By_StopGyro_Blocking(target, getAngleZ());
            nodesr.nowNode.angle = target;
        }

        /* 黑门被拦截：未实际通过，不改变扫描朝向/进门次序。 */
        return TRAFFIC_ROUTE_STATUS_OK;
    }
    else
    {
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
    }

    /* 已经第一次成功地过了某个门：之后所有门都朝左扫描。 */
    first_gate_passed = 1u;
    if (door_scan_count < 0xFFu)
        door_scan_count++;
    return TRAFFIC_ROUTE_STATUS_OK;
#endif
}
#else
TrafficRouteStatus_t TrafficRoute_HandleDoor(void)
{
    return TRAFFIC_ROUTE_STATUS_SCAN_FAILED;
}
#endif
