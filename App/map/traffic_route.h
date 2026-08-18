#ifndef TRAFFIC_ROUTE_H
#define TRAFFIC_ROUTE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TRAFFIC_ROUTE_DEFAULT_CLUE_A 5u
#define TRAFFIC_ROUTE_DEFAULT_CLUE_B 7u
#define TRAFFIC_ROUTE_GATE_COUNT     4u
#define TRAFFIC_ROUTE_GATE_INVALID   0xFFu   /* 尚无已确认可通行的门 */

typedef enum {
    TRAFFIC_ROUTE_COLOR_NONE = 0,
    TRAFFIC_ROUTE_COLOR_GREEN = 1,
    TRAFFIC_ROUTE_COLOR_BLUE = 2,
    TRAFFIC_ROUTE_COLOR_BLACK = 3
} TrafficRouteColor_t;

/* 门的来向：FORWARD=从 N3/N5 侧向外过门，RETURN=从远端(N8/N10/N12)侧回程过门。
 * 8 个方向(4 门对 × 2 来向)各自独立识别；行驶方向由当前门边判定，扫描侧
 * 仍按“第一次实际过门前全右、之后全左”的实车标定规则。 */
typedef enum {
    GATE_DIR_FORWARD = 0,
    GATE_DIR_RETURN
} GateDirection_t;

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
uint8_t TrafficRoute_GetLastColor(void);
TrafficRouteStatus_t TrafficRoute_HandleDoor(void);

/* 第一轮正向 GREEN/BLUE 首次成功通过的门（0=D2 1=D3 2=D4 3=D5；0xFF=未确认）。 */
uint8_t TrafficRoute_GetFirstPassableGate(void);

/* 第二轮是否旁路第一轮已确认可通行的门：map.routetime==2 且当前门 == first_passable_gate 时
 * 返回 1，由 Barrier_Door 直接放行（不扫描、不等门、不改线）。 */
uint8_t TrafficRoute_ShouldBypassDoor(void);

#ifdef __cplusplus
}
#endif

#endif
