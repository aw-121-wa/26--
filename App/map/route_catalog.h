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

/* 第二轮路线：使用第一轮已确认可通行的门，最短侧去珠峰 P7 → 南极 P8 → 同门返回 P2。
 * gate：0=D2 1=D3 2=D4 3=D5。当前仅支持 D3(1)/D4(2)，其他返回 NULL。 */
const uint8_t *RouteCatalog_GetRound2Fast(uint8_t gate);

#ifdef __cplusplus
}
#endif

#endif
