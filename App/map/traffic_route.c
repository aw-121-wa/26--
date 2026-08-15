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
static uint8_t door_scan_count = 0u;

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
        gate_colors[i] = TRAFFIC_ROUTE_COLOR_NONE;
    door_scan_count = 0u;
}

uint8_t TrafficRoute_GetGateColor(uint8_t gate_index)
{
    if (gate_index >= TRAFFIC_ROUTE_GATE_COUNT)
        return TRAFFIC_ROUTE_COLOR_NONE;
    return gate_colors[gate_index];
}

#ifndef TRAFFIC_ROUTE_UNIT_TEST
static TrafficRouteColor_t choose_effective_color(TrafficRouteColor_t left,
                                                  TrafficRouteColor_t right)
{
    if (left == TRAFFIC_ROUTE_COLOR_BLACK || right == TRAFFIC_ROUTE_COLOR_BLACK)
        return TRAFFIC_ROUTE_COLOR_BLACK;
    if (left == TRAFFIC_ROUTE_COLOR_BLUE || right == TRAFFIC_ROUTE_COLOR_BLUE)
        return TRAFFIC_ROUTE_COLOR_BLUE;
    if (left == TRAFFIC_ROUTE_COLOR_GREEN || right == TRAFFIC_ROUTE_COLOR_GREEN)
        return TRAFFIC_ROUTE_COLOR_GREEN;
    return TRAFFIC_ROUTE_COLOR_NONE;
}

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
    VisionPairResult_t pair;
    TrafficRouteColor_t left;
    TrafficRouteColor_t right;
    TrafficRouteColor_t effective;
    TrafficRouteStep_t step;
    uint8_t gate_index;
    uint8_t route_number;
    const uint8_t *segment;
    RouteBuildStatus_t status;

    if (Vision_ScanTrafficPair(&pair) != VISION_STATUS_OK)
        return TRAFFIC_ROUTE_STATUS_SCAN_FAILED;

    left = TrafficRoute_NormalizeVisionColor(pair.left.value);
    right = TrafficRoute_NormalizeVisionColor(pair.right.value);
    effective = choose_effective_color(left, right);

    if (effective == TRAFFIC_ROUTE_COLOR_NONE)
        return TRAFFIC_ROUTE_STATUS_NO_CHANGE;

    gate_index = current_gate_index();
    if (gate_index < TRAFFIC_ROUTE_GATE_COUNT)
        gate_colors[gate_index] = (uint8_t)effective;

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
