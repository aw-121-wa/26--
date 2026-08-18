#include "route_catalog.h"
#include "map.h"

#define LEGACY_DOOR_ROUTE(number, ...) \
    static const uint8_t door_route_##number[] = { __VA_ARGS__ }

/* 2023 national competition route catalog, retained as immutable fragments.
 * 每条 door_route 都是“门改线 → 远端 → 回 P2”的完整路径：
 * 末端(P7/P8/N8)统一续接回 P2，保证改线后最终回到起点/终点 P2。 */
LEGACY_DOOR_ROUTE(1, P5,N13,N12,N16,N18,B5,N19,C6,B7,C9,N22,C10,P7,C10,N22,B6,N20,P8,N20,C4,C8,C7,N14,C3,N9,N10,D5,N3,N4,B3,N2,P2,ROUTE_END);
LEGACY_DOOR_ROUTE(2, P5,N13,N12,N16,N18,B5,N19,C6,B7,C9,N22,B6,N20,P8,N20,B6,N22,C9,N22,C10,P7,C10,N22,B7,C6,N19,B5,N18,N16,N12,D2,N5,N4,B3,N2,P2,ROUTE_END);
LEGACY_DOOR_ROUTE(3, N12, N13,P5,N13,N12,N16,N18,B5,N19,C6,B7,C9,N22,C10,P7,C10,N22,B6,N20,P8,N20,C4,C8,C7,N14,C3,N9,N10,D5,N3,N4,B3,N2,P2,ROUTE_END);
LEGACY_DOOR_ROUTE(4, N12, N13,P5,N13,N12,N16,N18,B5,N19,C6,B7,C9,N22,B6,N20,P8,N20,B6,N22,C9,N22,C10,P7,C10,N22,B7,C6,N19,B5,N18,N16,N12,D2,N5,N4,B3,N2,P2,ROUTE_END);
LEGACY_DOOR_ROUTE(5, N9,B9,N7,P6,N7,B8,N9,C3,N14,C7,C8,C4,N20,B6,N22,C9,N22,C10,P7,G1,C9,N22,B6,N20,P8,N20,C4,C8,C7,N14,C3,N9,N10,D5,N3,N4,B3,N2,P2,ROUTE_END);
LEGACY_DOOR_ROUTE(6, N9,B9,N7,P6,N7,B8,N9,C3,N14,S3,N14,C7,C8,C4,N20,P8,N20,B6,N22,C9,N22,C10,P7,C10,N22,B7,C6,N19,B5,N18,N16,N12,D2,N5,N4,B3,N2,P2,ROUTE_END);
LEGACY_DOOR_ROUTE(7, N10,N9,B9,N7,P6,N7,B8,N9,C3,N14,C7,C8,C4,N20,B6,N22,C9,N22,C10,P7,G1,C9,N22,B6,N20,P8,N20,C4,C8,C7,N14,C3,N9,N10,D5,N3,N4,B3,N2,P2,ROUTE_END);
LEGACY_DOOR_ROUTE(8, N10,N9,B9,N7,P6,N7,B8,N9,C3,N14,C7,C8,C4,N20,P8,N20,B6,N22,C9,N22,C10,P7,C10,N22,B7,C6,N19,B5,N18,N16,N12,D2,N5,N4,B3,N2,P2,ROUTE_END);
LEGACY_DOOR_ROUTE(9, N4,N3,D4,N8,D3,N5,N4,B3,N2,P2,ROUTE_END);
LEGACY_DOOR_ROUTE(10, N4,N5,D3,N8,D3,N5,N4,B3,N2,P2,ROUTE_END);
LEGACY_DOOR_ROUTE(11, N10,N11,N12,P5,N13,N12,N16,N18,B5,N19,C6,B7,C9,N22,C10,P7,G1,C9,N22,B6,N20,P8,N20,C4,C8,C7,N14,C3,N9,N10,D5,N3,N4,B3,N2,P2,ROUTE_END);
LEGACY_DOOR_ROUTE(12, N10,N11,N12,P5,N13,N12,N16,N18,B5,N19,C6,B7,C9,N22,B6,N20,P8,N20,B6,N22,C9,N22,C10,P7,C10,N22,B7,C6,N19,B5,N18,N16,N12,D2,N5,N4,B3,N2,P2,ROUTE_END);
LEGACY_DOOR_ROUTE(13, N12,N11,N10,N9,B9,N7,P6,N7,B8,N9,C3,N14,C7,C8,C4,N20,B6,N22,C9,N22,C10,P7,G1,C9,N22,B6,N20,P8,N20,C4,C8,C7,N14,C3,N9,N10,D5,N3,N4,B3,N2,P2,ROUTE_END);
LEGACY_DOOR_ROUTE(14, N12,N11,N10,N9,B9,N7,P6,N7,B8,N9,C3,N14,C7,C8,C4,N20,P8,N20,B6,N22,C9,N22,C10,P7,C10,N22,B7,C6,N19,B5,N18,N16,N12,D2,N5,N4,B3,N2,P2,ROUTE_END);

static const uint8_t *const door_routes[ROUTE_CATALOG_DOOR_COUNT] = {
    door_route_1, door_route_2, door_route_3, door_route_4, door_route_5, door_route_6, door_route_7, door_route_8, door_route_9, door_route_10, door_route_11, door_route_12, door_route_13, door_route_14
};

const uint8_t *RouteCatalog_GetDoor(uint8_t route_number)
{
    if (route_number == 0u || route_number > ROUTE_CATALOG_DOOR_COUNT)
        return 0;
    return door_routes[route_number - 1u];
}

/* ======================== 第一轮门前向可通行高分段（P5→南极P7→珠峰P8→回家） ======================== */

/* 去珠峰主干（到 P8 结束）：P5→N13→N18→B5→N19→C6→B7→C9→N22→C10→P7(南极)
 * → C10→N22→B6→N20→P8(珠峰)。由 splice_forward_door_route 自动 prepend 远端。 */
#define ROUTE_TO_HIGH_SCORE \
    N12, N13, P5, \
    N13, N18, B5, N19, C6, B7, C9, N22, \
    C10, P7, \
    C10, N22, B6, N20, P8

/* 去珠峰主干“从 N13 开始”的变体（D2 正向 prepend N12，需从 N13 续，避免 N12→N12 重复） */
#define ROUTE_HIGH_FROM_N13 \
    N13, P5, \
    N13, N18, B5, N19, C6, B7, C9, N22, \
    C10, P7, \
    C10, N22, B6, N20, P8

/* 珠峰上侧返程主干（P8 之后从 N20 回到 N10 门区侧） */
#define ROUTE_P8_RETURN_UPPER \
    N20, C4, C8, C7, N14, C3, N9, N10

/* 第一轮去程（D3/D4/D5 共用，splice prepend N8/N10）；D5 是首选返程门 → N3→N4→B3→N2→P2。
 * D5 不可返程(BLUE-RETURN/BLACK)时由门处理层四门穷尽换门（N10→N12→D2→…）。 */
static const uint8_t round1_highscore[] = {
    ROUTE_TO_HIGH_SCORE,
    ROUTE_P8_RETURN_UPPER,
    D5, N3, N4, B3, N2, P2, ROUTE_END
};

/* 第一轮去程（D2 正向 prepend N12，续接从 N13 开始） */
static const uint8_t round1_highscore_d2[] = {
    ROUTE_HIGH_FROM_N13,
    ROUTE_P8_RETURN_UPPER,
    D5, N3, N4, B3, N2, P2, ROUTE_END
};

const uint8_t *RouteCatalog_GetRound1Forward(uint8_t gate_index)
{
    switch (gate_index)
    {
    case 0u: return round1_highscore_d2;              /* D2 */
    case 1u: /* D3 */
    case 2u: /* D4 */
    case 3u: /* D5 */
        return round1_highscore;
    default:
        return 0;
    }
}

/* ======================== 第二轮路线（复用第一轮已确认的 forward_gate / return_gate） ======================== */

/* 第二轮中间段：与第一轮同一条 → 珠峰(P8) → 上侧返回到 N10 */
#define MIDDLE_D4D3D5  ROUTE_TO_HIGH_SCORE, ROUTE_P8_RETURN_UPPER
#define MIDDLE_D2      ROUTE_HIGH_FROM_N13, ROUTE_P8_RETURN_UPPER

/* 正向出口段（round1 正向出口门：P2→…→门→远端） */
#define ENTRY_D4  N2, B3, N4, N3, D4, N8
#define ENTRY_D3  N2, B3, N4, N5, D3, N8
#define ENTRY_D5  N2, B3, N4, N3, D5, N10
#define ENTRY_D2  N2, B3, N4, N5, D2, N12

/* 返程尾段（round1 实际确认的返程门：D5 首选，D2/D3/D4 为四门穷尽换门后的合法返程门） */
#define BACK_D5  D5, N3, N4, B3, N2, P2
#define BACK_D2  N12, D2, N5, N4, B3, N2, P2
#define BACK_D3  N12, N8, D3, N5, N4, B3, N2, P2
#define BACK_D4  N12, N8, D4, N3, N4, B3, N2, P2

/* forward × return 全部组合（forward{2,1,3,0} × return{3,0,1,2}） */
static const uint8_t round2_d4_d5[] = { ENTRY_D4, MIDDLE_D4D3D5, BACK_D5, ROUTE_END };
static const uint8_t round2_d4_d2[] = { ENTRY_D4, MIDDLE_D4D3D5, BACK_D2, ROUTE_END };
static const uint8_t round2_d4_d3[] = { ENTRY_D4, MIDDLE_D4D3D5, BACK_D3, ROUTE_END };
static const uint8_t round2_d4_d4[] = { ENTRY_D4, MIDDLE_D4D3D5, BACK_D4, ROUTE_END };
static const uint8_t round2_d3_d5[] = { ENTRY_D3, MIDDLE_D4D3D5, BACK_D5, ROUTE_END };
static const uint8_t round2_d3_d2[] = { ENTRY_D3, MIDDLE_D4D3D5, BACK_D2, ROUTE_END };
static const uint8_t round2_d3_d3[] = { ENTRY_D3, MIDDLE_D4D3D5, BACK_D3, ROUTE_END };
static const uint8_t round2_d3_d4[] = { ENTRY_D3, MIDDLE_D4D3D5, BACK_D4, ROUTE_END };
static const uint8_t round2_d5_d5[] = { ENTRY_D5, MIDDLE_D4D3D5, BACK_D5, ROUTE_END };
static const uint8_t round2_d5_d2[] = { ENTRY_D5, MIDDLE_D4D3D5, BACK_D2, ROUTE_END };
static const uint8_t round2_d5_d3[] = { ENTRY_D5, MIDDLE_D4D3D5, BACK_D3, ROUTE_END };
static const uint8_t round2_d5_d4[] = { ENTRY_D5, MIDDLE_D4D3D5, BACK_D4, ROUTE_END };
static const uint8_t round2_d2_d5[] = { ENTRY_D2, MIDDLE_D2, BACK_D5, ROUTE_END };
static const uint8_t round2_d2_d2[] = { ENTRY_D2, MIDDLE_D2, BACK_D2, ROUTE_END };
static const uint8_t round2_d2_d3[] = { ENTRY_D2, MIDDLE_D2, BACK_D3, ROUTE_END };
static const uint8_t round2_d2_d4[] = { ENTRY_D2, MIDDLE_D2, BACK_D4, ROUTE_END };

const uint8_t *RouteCatalog_GetRound2Fast(uint8_t forward_gate,
                                          uint8_t return_gate)
{
    switch (forward_gate)
    {
    case 2u: /* D4 */
        switch (return_gate)
        {
        case 3u: return round2_d4_d5;
        case 0u: return round2_d4_d2;
        case 1u: return round2_d4_d3;
        case 2u: return round2_d4_d4;
        default: return 0;
        }
    case 1u: /* D3 */
        switch (return_gate)
        {
        case 3u: return round2_d3_d5;
        case 0u: return round2_d3_d2;
        case 1u: return round2_d3_d3;
        case 2u: return round2_d3_d4;
        default: return 0;
        }
    case 3u: /* D5 */
        switch (return_gate)
        {
        case 3u: return round2_d5_d5;
        case 0u: return round2_d5_d2;
        case 1u: return round2_d5_d3;
        case 2u: return round2_d5_d4;
        default: return 0;
        }
    case 0u: /* D2 */
        switch (return_gate)
        {
        case 3u: return round2_d2_d5;
        case 0u: return round2_d2_d2;
        case 1u: return round2_d2_d3;
        case 2u: return round2_d2_d4;
        default: return 0;
        }
    default:
        return 0;
    }
}
