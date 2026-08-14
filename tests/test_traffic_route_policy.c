#include "traffic_route.h"

#include <assert.h>
#include <stdint.h>

static void expect_route(uint8_t clue_a, uint8_t clue_b,
                         TrafficRouteStep_t step,
                         TrafficRouteColor_t color,
                         uint8_t expected)
{
    uint8_t actual = TrafficRoute_SelectDoorRouteNumber(clue_a, clue_b,
                                                        step, color);
    assert(actual == expected);
}

int main(void)
{
    assert(TrafficRoute_NormalizeVisionColor(1u) == TRAFFIC_ROUTE_COLOR_GREEN);
    assert(TrafficRoute_NormalizeVisionColor(2u) == TRAFFIC_ROUTE_COLOR_BLUE);
    assert(TrafficRoute_NormalizeVisionColor(3u) == TRAFFIC_ROUTE_COLOR_BLACK);
    assert(TrafficRoute_NormalizeVisionColor(99u) == TRAFFIC_ROUTE_COLOR_NONE);

    assert(TrafficRoute_IsColorPassable(TRAFFIC_ROUTE_COLOR_GREEN, 0u) == 1u);
    assert(TrafficRoute_IsColorPassable(TRAFFIC_ROUTE_COLOR_BLUE, 0u) == 1u);
    assert(TrafficRoute_IsColorPassable(TRAFFIC_ROUTE_COLOR_BLUE, 1u) == 0u);
    assert(TrafficRoute_IsColorPassable(TRAFFIC_ROUTE_COLOR_BLACK, 0u) == 0u);

    expect_route(5u, 7u, TRAFFIC_ROUTE_STEP_FIRST_PASSABLE,
                 TRAFFIC_ROUTE_COLOR_GREEN, 1u);
    expect_route(5u, 8u, TRAFFIC_ROUTE_STEP_FIRST_PASSABLE,
                 TRAFFIC_ROUTE_COLOR_GREEN, 2u);
    expect_route(6u, 7u, TRAFFIC_ROUTE_STEP_FIRST_PASSABLE,
                 TRAFFIC_ROUTE_COLOR_BLUE, 5u);
    expect_route(6u, 8u, TRAFFIC_ROUTE_STEP_FIRST_PASSABLE,
                 TRAFFIC_ROUTE_COLOR_BLUE, 6u);

    expect_route(5u, 7u, TRAFFIC_ROUTE_STEP_SECOND_PASSABLE,
                 TRAFFIC_ROUTE_COLOR_GREEN, 3u);
    expect_route(5u, 8u, TRAFFIC_ROUTE_STEP_SECOND_PASSABLE,
                 TRAFFIC_ROUTE_COLOR_GREEN, 4u);
    expect_route(6u, 7u, TRAFFIC_ROUTE_STEP_SECOND_PASSABLE,
                 TRAFFIC_ROUTE_COLOR_BLUE, 7u);
    expect_route(6u, 8u, TRAFFIC_ROUTE_STEP_SECOND_PASSABLE,
                 TRAFFIC_ROUTE_COLOR_BLUE, 8u);

    expect_route(5u, 7u, TRAFFIC_ROUTE_STEP_OUTER_FALLBACK,
                 TRAFFIC_ROUTE_COLOR_BLACK, 9u);
    expect_route(6u, 8u, TRAFFIC_ROUTE_STEP_OUTER_FALLBACK,
                 TRAFFIC_ROUTE_COLOR_BLACK, 10u);

    expect_route(5u, 7u, TRAFFIC_ROUTE_STEP_LATE_BLOCK,
                 TRAFFIC_ROUTE_COLOR_BLACK, 11u);
    expect_route(5u, 8u, TRAFFIC_ROUTE_STEP_LATE_BLOCK,
                 TRAFFIC_ROUTE_COLOR_BLACK, 12u);
    expect_route(6u, 7u, TRAFFIC_ROUTE_STEP_LATE_BLOCK,
                 TRAFFIC_ROUTE_COLOR_BLACK, 13u);
    expect_route(6u, 8u, TRAFFIC_ROUTE_STEP_LATE_BLOCK,
                 TRAFFIC_ROUTE_COLOR_BLACK, 14u);

    expect_route(5u, 6u, TRAFFIC_ROUTE_STEP_FIRST_PASSABLE,
                 TRAFFIC_ROUTE_COLOR_GREEN, 0u);
    expect_route(5u, 7u, TRAFFIC_ROUTE_STEP_FIRST_PASSABLE,
                 TRAFFIC_ROUTE_COLOR_BLACK, 0u);

    return 0;
}
