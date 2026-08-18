#include "route_catalog.h"
#include "map.h"

#define LEGACY_DOOR_ROUTE(number, ...) \
    static const uint8_t door_route_##number[] = { __VA_ARGS__ }

/* 2023 national competition route catalog, retained as immutable fragments.
 * 每条 door_route 都是“门改线 → 远端 → 回 P2”的完整路径：
 * 末端(P7/P8/N8)统一续接回 P2，保证改线后最终回到起点/终点 P2。 */
LEGACY_DOOR_ROUTE(1, P6,N13,N12,N16,N18,B5,N19,C6,B7,C9,N22,C10,P8,C10,N22,B6,N20,P7,N20,C4,C8,C7,N14,C3,N9,N10,D5,N3,N4,B3,N2,P2,ROUTE_END);
LEGACY_DOOR_ROUTE(2, P6,N13,N12,N16,N18,B5,N19,C6,B7,C9,N22,B6,N20,P7,N20,B6,N22,C9,N22,C10,P8,C10,N22,B7,C6,N19,B5,N18,N16,N12,D2,N5,N4,B3,N2,P2,ROUTE_END);
LEGACY_DOOR_ROUTE(3, N12, N13,P6,N13,N12,N16,N18,B5,N19,C6,B7,C9,N22,C10,P8,C10,N22,B6,N20,P7,N20,C4,C8,C7,N14,C3,N9,N10,D5,N3,N4,B3,N2,P2,ROUTE_END);
LEGACY_DOOR_ROUTE(4, N12, N13,P6,N13,N12,N16,N18,B5,N19,C6,B7,C9,N22,B6,N20,P7,N20,B6,N22,C9,N22,C10,P8,C10,N22,B7,C6,N19,B5,N18,N16,N12,D2,N5,N4,B3,N2,P2,ROUTE_END);
LEGACY_DOOR_ROUTE(5, N9,B9,N7,P5,N7,B8,N9,C3,N14,C7,C8,C4,N20,B6,N22,C9,N22,C10,P8,G1,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,C3,N9,N10,D5,N3,N4,B3,N2,P2,ROUTE_END);
LEGACY_DOOR_ROUTE(6, N9,B9,N7,P5,N7,B8,N9,C3,N14,S3,N14,C7,C8,C4,N20,P7,N20,B6,N22,C9,N22,C10,P8,C10,N22,B7,C6,N19,B5,N18,N16,N12,D2,N5,N4,B3,N2,P2,ROUTE_END);
LEGACY_DOOR_ROUTE(7, N10,N9,B9,N7,P5,N7,B8,N9,C3,N14,C7,C8,C4,N20,B6,N22,C9,N22,C10,P8,G1,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,C3,N9,N10,D5,N3,N4,B3,N2,P2,ROUTE_END);
LEGACY_DOOR_ROUTE(8, N10,N9,B9,N7,P5,N7,B8,N9,C3,N14,C7,C8,C4,N20,P7,N20,B6,N22,C9,N22,C10,P8,C10,N22,B7,C6,N19,B5,N18,N16,N12,D2,N5,N4,B3,N2,P2,ROUTE_END);
LEGACY_DOOR_ROUTE(9, N4,N3,D4,N8,D3,N5,N4,B3,N2,P2,ROUTE_END);
LEGACY_DOOR_ROUTE(10, N4,N5,D3,N8,D3,N5,N4,B3,N2,P2,ROUTE_END);
LEGACY_DOOR_ROUTE(11, N10,N11,N12,P6,N13,N12,N16,N18,B5,N19,C6,B7,C9,N22,C10,P8,G1,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,C3,N9,N10,D5,N3,N4,B3,N2,P2,ROUTE_END);
LEGACY_DOOR_ROUTE(12, N10,N11,N12,P6,N13,N12,N16,N18,B5,N19,C6,B7,C9,N22,B6,N20,P7,N20,B6,N22,C9,N22,C10,P8,C10,N22,B7,C6,N19,B5,N18,N16,N12,D2,N5,N4,B3,N2,P2,ROUTE_END);
LEGACY_DOOR_ROUTE(13, N12,N11,N10,N9,B9,N7,P5,N7,B8,N9,C3,N14,C7,C8,C4,N20,B6,N22,C9,N22,C10,P8,G1,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,C3,N9,N10,D5,N3,N4,B3,N2,P2,ROUTE_END);
LEGACY_DOOR_ROUTE(14, N12,N11,N10,N9,B9,N7,P5,N7,B8,N9,C3,N14,C7,C8,C4,N20,P7,N20,B6,N22,C9,N22,C10,P8,C10,N22,B7,C6,N19,B5,N18,N16,N12,D2,N5,N4,B3,N2,P2,ROUTE_END);

static const uint8_t *const door_routes[ROUTE_CATALOG_DOOR_COUNT] = {
    door_route_1, door_route_2, door_route_3, door_route_4, door_route_5, door_route_6, door_route_7, door_route_8, door_route_9, door_route_10, door_route_11, door_route_12, door_route_13, door_route_14
};

const uint8_t *RouteCatalog_GetDoor(uint8_t route_number)
{
    if (route_number == 0u || route_number > ROUTE_CATALOG_DOOR_COUNT)
        return 0;
    return door_routes[route_number - 1u];
}

/* ======================== 第一轮门前向可通行短段 ======================== */

/* D4 通过后：D4→N8(自动prepend)→N12→N13→P6 → 原路返回(N12→N8→D4)→N3→N4→B3→N2→P2 */
static const uint8_t round1_d4_p6_return[] = {
    N12, N13, P6, N13, N12, N8, D4, N3, N4, B3, N2, P2, ROUTE_END
};

/* D3 通过后：D3→N8(自动prepend)→N12→N13→P6 → 原路返回(N12→N8→D3)→N5→N4→B3→N2→P2 */
static const uint8_t round1_d3_p6_return[] = {
    N12, N13, P6, N13, N12, N8, D3, N5, N4, B3, N2, P2, ROUTE_END
};

const uint8_t *RouteCatalog_GetRound1Forward(uint8_t gate_index)
{
    switch (gate_index)
    {
    case 2u: return round1_d4_p6_return;   /* D4 */
    case 1u: return round1_d3_p6_return;   /* D3 */
    default: return 0;
    }
}

/* ======================== 第二轮路线（最短侧去珠峰/南极） ======================== */

/* 珠峰/南极共用中间段：N8→N10→N9→C3→N14→C7→C8→C4→N20→P7 → N20→B6→N22→C10→P8
 * → C10→N22→B7→C6→N19→B5→N18→N16→N12→N8（最短侧，全部连接已对照地图校验） */
#define ROUND2_MIDDLE_N10 N10, N9, C3, N14, C7, C8, C4, N20, P7, \
                          N20, B6, N22, C10, P8, \
                          C10, N22, B7, C6, N19, B5, N18, N16, N12, N8

/* 正向出口段 */
#define ROUND2_ENTRY_D4  N2, B3, N4, N3, D4, N8
#define ROUND2_ENTRY_D3  N2, B3, N4, N5, D3, N8

/* 返程尾段（返程门） */
#define ROUND2_BACK_D4   D4, N3, N4, B3, N2, P2
#define ROUND2_BACK_D3   D3, N5, N4, B3, N2, P2

/* D4出去 / D4回来（BLUE双向） */
static const uint8_t round2_d4_d4[] = {
    ROUND2_ENTRY_D4, ROUND2_MIDDLE_N10, ROUND2_BACK_D4, ROUTE_END
};

/* D4出去 / D3回来（D4 GREEN 出，D3 BLUE 回） */
static const uint8_t round2_d4_d3[] = {
    ROUND2_ENTRY_D4, ROUND2_MIDDLE_N10, ROUND2_BACK_D3, ROUTE_END
};

/* D3出去 / D3回来（BLUE双向） */
static const uint8_t round2_d3_d3[] = {
    ROUND2_ENTRY_D3, ROUND2_MIDDLE_N10, ROUND2_BACK_D3, ROUTE_END
};

/* D3出去 / D4回来（D3 GREEN 出，D4 BLUE 回） */
static const uint8_t round2_d3_d4[] = {
    ROUND2_ENTRY_D3, ROUND2_MIDDLE_N10, ROUND2_BACK_D4, ROUTE_END
};

const uint8_t *RouteCatalog_GetRound2Fast(uint8_t forward_gate,
                                          uint8_t return_gate)
{
    if (forward_gate == 2u && return_gate == 2u) return round2_d4_d4; /* D4出 / D4回 */
    if (forward_gate == 2u && return_gate == 1u) return round2_d4_d3; /* D4出 / D3回 */
    if (forward_gate == 1u && return_gate == 1u) return round2_d3_d3; /* D3出 / D3回 */
    if (forward_gate == 1u && return_gate == 2u) return round2_d3_d4; /* D3出 / D4回 */
    return 0;
}
