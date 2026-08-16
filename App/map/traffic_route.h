#ifndef TRAFFIC_ROUTE_H
#define TRAFFIC_ROUTE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TRAFFIC_ROUTE_DEFAULT_CLUE_A 5u
#define TRAFFIC_ROUTE_DEFAULT_CLUE_B 7u
#define TRAFFIC_ROUTE_GATE_COUNT     4u

typedef enum {
    TRAFFIC_ROUTE_COLOR_NONE = 0,
    TRAFFIC_ROUTE_COLOR_GREEN = 1,
    TRAFFIC_ROUTE_COLOR_BLUE = 2,
    TRAFFIC_ROUTE_COLOR_BLACK = 3
} TrafficRouteColor_t;

typedef enum {
    TRAFFIC_ROUTE_STEP_FIRST_PASSABLE = 0,
    TRAFFIC_ROUTE_STEP_SECOND_PASSABLE,
    TRAFFIC_ROUTE_STEP_OUTER_FALLBACK,
    TRAFFIC_ROUTE_STEP_LATE_BLOCK
} TrafficRouteStep_t;

typedef enum {
    TRAFFIC_ROUTE_STATUS_OK = 0,
    TRAFFIC_ROUTE_STATUS_NO_CHANGE,
    TRAFFIC_ROUTE_STATUS_SCAN_FAILED,
    TRAFFIC_ROUTE_STATUS_NO_ROUTE,
    TRAFFIC_ROUTE_STATUS_SPLICE_FAILED
} TrafficRouteStatus_t;

TrafficRouteColor_t TrafficRoute_NormalizeVisionColor(uint8_t vision_value);
uint8_t TrafficRoute_IsColorPassable(TrafficRouteColor_t color,
                                     uint8_t is_return_trip);
uint8_t TrafficRoute_SelectDoorRouteNumber(uint8_t clue_a, uint8_t clue_b,
                                           TrafficRouteStep_t step,
                                           TrafficRouteColor_t color);
void TrafficRoute_Reset(void);
uint8_t TrafficRoute_GetGateColor(uint8_t gate_index);
uint8_t TrafficRoute_GetLastColor(void);
TrafficRouteStatus_t TrafficRoute_HandleDoor(void);

#ifdef __cplusplus
}
#endif

#endif
