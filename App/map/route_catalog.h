#ifndef ROUTE_CATALOG_H
#define ROUTE_CATALOG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
#define ROUTE_CATALOG_DOOR_COUNT 14u

const uint8_t *RouteCatalog_GetDoor(uint8_t route_number);

/* 第一轮门正向可通行：返回去 P5 → 南极 P7 → 珠峰 P8 → 回家 的高分路线
 * （以远端前的第一节点开头，由 splice_forward_door_route 自动 prepend 远端；
 * 本段不再以远端开头）。gate_index：0=D2 1=D3 2=D4 3=D5，四扇门均支持。 */
const uint8_t *RouteCatalog_GetRound1Forward(uint8_t gate_index);

/* 第二轮路线：中间段与第一轮相同（P5→南极P7→珠峰P8→上侧回 N10），再走返程门回 P2。
 * 支持 forward_gate / return_gate ∈ {D2(0),D3(1),D4(2),D5(3)} 全部组合。
 * 其余无效组合返回 NULL。 */
const uint8_t *RouteCatalog_GetRound2Fast(uint8_t forward_gate,
                                          uint8_t return_gate);

#ifdef __cplusplus
}
#endif

#endif
