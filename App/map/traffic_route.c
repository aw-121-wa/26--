#include "traffic_route.h"

#ifndef TRAFFIC_ROUTE_UNIT_TEST
#include "map.h"
#include "route_catalog.h"
#include "../vision/vision_api.h"
#endif

#define TRAFFIC_ROUTE_NO_ROUTE 0u

/* 临时开关：置1禁用视觉红绿灯（跳过扫描与动态改线，纯跑原路线）；置0恢复。 */
#define TRAFFIC_ROUTE_VISION_DISABLED 0

static uint8_t gate_colors[TRAFFIC_ROUTE_GATE_COUNT] = {0u, 0u, 0u, 0u};
/* 每个门已确定的红绿灯颜色：非 NONE 表示已存下，之后直接复用不再扫描。 */
static uint8_t gate_stored_color[TRAFFIC_ROUTE_GATE_COUNT] = {0u, 0u, 0u, 0u};
/* 是否已第一次经过某个门：为0时所有门朝右扫描，一旦第一次过了门就置1，之后所有门朝左。 */
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

void TrafficRoute_Reset(void)
{
    uint8_t i;

    for (i = 0u; i < TRAFFIC_ROUTE_GATE_COUNT; i++)
    {
        gate_colors[i] = TRAFFIC_ROUTE_COLOR_NONE;
        gate_stored_color[i] = TRAFFIC_ROUTE_COLOR_NONE;
    }
    first_gate_passed = 0u;
    door_scan_count = 0u;
    last_effective_color = TRAFFIC_ROUTE_COLOR_NONE;
}

uint8_t TrafficRoute_GetGateColor(uint8_t gate_index)
{
    if (gate_index >= TRAFFIC_ROUTE_GATE_COUNT)
        return TRAFFIC_ROUTE_COLOR_NONE;
    return gate_colors[gate_index];
}

uint8_t TrafficRoute_GetLastColor(void)
{
    return last_effective_color;
}

#ifndef TRAFFIC_ROUTE_UNIT_TEST
static TrafficRouteStep_t select_step(TrafficRouteColor_t color)
{
    if (color == TRAFFIC_ROUTE_COLOR_BLACK)
    {
        if (door_scan_count == 0u)
            return TRAFFIC_ROUTE_STEP_OUTER_FALLBACK;
        return TRAFFIC_ROUTE_STEP_LATE_BLOCK;
    }

    if (door_scan_count == 0u)
        return TRAFFIC_ROUTE_STEP_FIRST_PASSABLE;
    return TRAFFIC_ROUTE_STEP_SECOND_PASSABLE;
}

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

static TrafficRouteStep_t select_step_for_gate(uint8_t gate_index,
                                               TrafficRouteColor_t color)
{
    if (gate_index == 0u || gate_index == 3u)
    {
        return color == TRAFFIC_ROUTE_COLOR_BLACK ?
               TRAFFIC_ROUTE_STEP_OUTER_FALLBACK :
               TRAFFIC_ROUTE_STEP_FIRST_PASSABLE;
    }
    if (gate_index == 1u || gate_index == 2u)
    {
        return color == TRAFFIC_ROUTE_COLOR_BLACK ?
               TRAFFIC_ROUTE_STEP_LATE_BLOCK :
               TRAFFIC_ROUTE_STEP_SECOND_PASSABLE;
    }
    return select_step(color);
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

    /* 已存下该门信息：直接复用，不再动舵机扫描，也不再二次拼接。
     * 完整 door 路线已含回 P2 全程，二次经过该门时保持不动即可。 */
    if (gate_index < TRAFFIC_ROUTE_GATE_COUNT &&
        gate_stored_color[gate_index] != TRAFFIC_ROUTE_COLOR_NONE)
    {
        last_effective_color = gate_stored_color[gate_index];
        return TRAFFIC_ROUTE_STATUS_NO_CHANGE;
    }

    /* 未存下信息：单边扫描。第一次经过门之前所有门朝右，
     * 第一次经过门之后所有门朝左。 */
    direction = first_gate_passed ? VISION_DIRECTION_LEFT
                                  : VISION_DIRECTION_RIGHT;
    if (Vision_ScanSingleSide(direction, &sample) != VISION_STATUS_OK)
        return TRAFFIC_ROUTE_STATUS_SCAN_FAILED;

    effective = TrafficRoute_NormalizeVisionColor(sample.value);
    last_effective_color = (uint8_t)effective;

    if (effective == TRAFFIC_ROUTE_COLOR_NONE)
        return TRAFFIC_ROUTE_STATUS_NO_CHANGE;

    if (gate_index < TRAFFIC_ROUTE_GATE_COUNT)
    {
        gate_colors[gate_index] = (uint8_t)effective;
        gate_stored_color[gate_index] = (uint8_t)effective; /* 存下来，后续复用 */
    }

    step = select_step_for_gate(gate_index, effective);
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
