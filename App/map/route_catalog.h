#ifndef ROUTE_CATALOG_H
#define ROUTE_CATALOG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
#define ROUTE_CATALOG_DOOR_COUNT 14u

const uint8_t *RouteCatalog_GetDoor(uint8_t route_number);

/* 第一轮门正向可通行：返回去 P6 平台后原路返回 P2 的短段（以远端前的第一节点开头，
 * 由 splice_forward_door_route 自动 prepend 远端 N8，本段不再以 N8 开头）。
 * gate_index：0=D2 1=D3 2=D4 3=D5。当前仅支持 round1 会经过的 D3(1)/D4(2)。 */
const uint8_t *RouteCatalog_GetRound1Forward(uint8_t gate_index);

/* 第二轮路线：使用第一轮已确认的“正向出口门”与“返程门”，最短侧去珠峰 P7 → 南极 P8 →
 * 再从返程门返回 P2。forward_gate/return_gate：0=D2 1=D3 2=D4 3=D5。
 * 当前仅支持 D3(1)/D4(2) 的 4 种组合（出去/回来）；其余返回 NULL。 */
const uint8_t *RouteCatalog_GetRound2Fast(uint8_t forward_gate,
                                          uint8_t return_gate);

#ifdef __cplusplus
}
#endif

#endif
