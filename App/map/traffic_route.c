#include "traffic_route.h"

#ifndef TRAFFIC_ROUTE_UNIT_TEST
#include <math.h>
#include "map.h"
#include "route_catalog.h"
#include "imu.h"
#include "../vision/vision_api.h"
#include "../chassis/chassis_api.h"
#include "../../Task/motor_task.h"
#include "scaner.h"
#include "FreeRTOS.h"
#include "task.h"
#endif

#define TRAFFIC_ROUTE_NO_ROUTE 0u

/* 临时开关：置1禁用视觉红绿灯（跳过扫描与动态改线，纯跑原路线）；置0恢复。 */
#define TRAFFIC_ROUTE_VISION_DISABLED 0

/* 最近一次扫描识别的颜色：仅用于 HMI 显示。 */
static uint8_t last_effective_color = TRAFFIC_ROUTE_COLOR_NONE;

#ifndef TRAFFIC_ROUTE_UNIT_TEST
/* 扫描方向启发式（实车标定规则）：第一次实际通过门之前，所有门都朝右扫；
 * 第一次通过门之后，所有门朝左扫。与门对/来向无关的全局行经方向规则。 */
static uint8_t gate_first_passed = 0u;

/* 第一轮门状态缓存：
 *   forward_gate — 第一轮正向实际通过的门(0-3)
 *   return_gate  — 第一轮返程实际通过的门(0-3)
 * 两者都未确认(TRAFFIC_ROUTE_GATE_INVALID) → 第二轮无合法路线。 */
static uint8_t forward_gate = TRAFFIC_ROUTE_GATE_INVALID;
static uint8_t return_gate = TRAFFIC_ROUTE_GATE_INVALID;

/* 本次返程门搜索期间已确认“不可返程”的门位掩码：已 blocked 的门不得再次成为候选，
 * 防止 D4↔D3 无限换门；无其他合法门时安全停车。 */
static uint8_t return_blocked_mask = 0u;

/* 本次正向(出口)换门搜索期间已确认“不可通行”的门位掩码：BLACK 或换门失败的出口门
 * 不得再次成为候选，防 D4↔D3 无限换门；无其他合法出口门时安全停车。 */
static uint8_t forward_blocked_mask = 0u;

/* 连续黑门换门计数防护（按 门对×来向 分槽）：同层双阻塞会反复换门，加次数上限
 * 避免死循环 + splice 累积失败。达到上限说明当前门(方向)确认不可通行 → 安全停车。
 * 每槽在一次换路链内累计；换路成功/正常通行后复位。 */
#define GATE_SWAP_MAX 3u
static uint8_t gate_swap_count[TRAFFIC_ROUTE_GATE_COUNT][2u] = {{0u, 0u}, {0u, 0u}, {0u, 0u}, {0u, 0u}};

static void gate_swap_reset(uint8_t gate_index, uint8_t dir)
{
    if (gate_index < TRAFFIC_ROUTE_GATE_COUNT && dir <= (uint8_t)GATE_DIR_RETURN)
        gate_swap_count[gate_index][dir] = 0u;
}
#endif

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

/* 官方门规则：BLACK 禁止；GREEN 双向；BLUE 仅 FORWARD 可通、RETURN 禁止；
 * NONE/扫描失败不可通行。is_return_trip：0=正向，1=返程。 */
uint8_t TrafficRoute_IsColorPassable(TrafficRouteColor_t color,
                                     uint8_t is_return_trip)
{
    if (color == TRAFFIC_ROUTE_COLOR_BLACK)
        return 0u;
    if (color == TRAFFIC_ROUTE_COLOR_GREEN)
        return 1u;
    if (color == TRAFFIC_ROUTE_COLOR_BLUE)
        return is_return_trip ? 0u : 1u;
    return 0u;   /* NONE/无法识别：不可通行 */
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

uint8_t TrafficRoute_GetLastColor(void)
{
    return last_effective_color;
}

#ifndef TRAFFIC_ROUTE_UNIT_TEST
/* 前向声明：识别当前门对与来向（定义在下方） */
static uint8_t current_gate(uint8_t *dir);

/* 第一轮正向实际通过的门（供第二轮入口选路/旁路）。 */
uint8_t TrafficRoute_GetForwardGate(void)
{
    return forward_gate;
}

/* 第一轮实际确认可返程通过的门（供第二轮返程选路/旁路）。 */
uint8_t TrafficRoute_GetReturnGate(void)
{
    return return_gate;
}

/* 第二轮是否旁路该门：FORWARD 只旁路 forward_gate，RETURN 只旁路 return_gate。
 * 这两个门都是第一轮实际按对应方向通过过的（GREEN 记录方向一致，或 BLUE），
 * 故二轮可直接按历史方向旁路，无需重新扫描。 */
uint8_t TrafficRoute_ShouldBypassDoor(void)
{
    uint8_t dir = (uint8_t)GATE_DIR_FORWARD;
    uint8_t gate_index;

    if (map.routetime != 2u)
        return 0u;

    gate_index = current_gate(&dir);
    if (gate_index >= TRAFFIC_ROUTE_GATE_COUNT)
        return 0u;

    if (dir == GATE_DIR_FORWARD)
        return (gate_index == forward_gate) ? 1u : 0u;
    else /* GATE_DIR_RETURN */
    {
        if (return_gate == TRAFFIC_ROUTE_GATE_INVALID)
            return 0u;
        return (gate_index == return_gate) ? 1u : 0u;
    }
}

/* 统一通行判定：BLACK 禁止；GREEN 双向；BLUE 仅 FORWARD 可通、RETURN 禁止；
 * NONE/无法识别不可通行（NONE 在 HandleDoor 中单独短路，此处仅作兜底）。 */
static uint8_t gate_can_pass(uint8_t gate_index, TrafficRouteColor_t color,
                             uint8_t dir)
{
    (void)gate_index;
    if (color == TRAFFIC_ROUTE_COLOR_BLACK)
        return 0u;
    if (color == TRAFFIC_ROUTE_COLOR_GREEN)
        return 1u;
    if (color == TRAFFIC_ROUTE_COLOR_BLUE)
        return (dir == GATE_DIR_FORWARD) ? 1u : 0u;
    return 0u;   /* NONE */
}
/* 识别当前过门边属于哪个门对，并同时输出来向(FORWARD/RETURN)。
 * 门已实体化为 D2~D5 独立节点：nowNode 即门节点；方向由 lastNode(来向端点)判断。
 * FORWARD=从内层端点(N5/N3)进入，RETURN=从外层端点(N12/N8/N10)进入。
 * 返回 0..3=门对号，0xFF=非门边。 */
static uint8_t current_gate(uint8_t *dir)
{
    if (dir == 0)
        return 0xFFu;

    switch (nodesr.nowNode.nodenum)
    {
    case D2:   /* 门2：N5 ↔ N12 */
        *dir = (nodesr.lastNode.nodenum == N5) ? (uint8_t)GATE_DIR_FORWARD : (uint8_t)GATE_DIR_RETURN;
        return 0u;
    case D3:   /* 门3：N5 ↔ N8 */
        *dir = (nodesr.lastNode.nodenum == N5) ? (uint8_t)GATE_DIR_FORWARD : (uint8_t)GATE_DIR_RETURN;
        return 1u;
    case D4:   /* 门4：N3 ↔ N8 */
        *dir = (nodesr.lastNode.nodenum == N3) ? (uint8_t)GATE_DIR_FORWARD : (uint8_t)GATE_DIR_RETURN;
        return 2u;
    case D5:   /* 门5：N3 ↔ N10 */
        *dir = (nodesr.lastNode.nodenum == N3) ? (uint8_t)GATE_DIR_FORWARD : (uint8_t)GATE_DIR_RETURN;
        return 3u;
    default:
        return 0xFFu;
    }
}


/* 阻塞门换线（BLACK 任意方向，或 BLUE 返程单向禁止时复用）：退回来路源节点，
 * 改走同层对面那扇门重新进入，并保留原路线重入门后的后半段。
 * 换过去若仍为阻塞，到达该门时会重新识别并再次换线；同层双阻塞则由
 * forward/return_blocked_mask 检测到后安全停车（绝不 D4↔D3 无限换门）。
 * 门已独立成节点：退回源节点即 lastNode（src）；优先传感倒车，兜底按实际进距对称倒回。 */
/* 阻塞门短倒车比例：带 BLACK_REVERSE_SHORT 标志的门（当前仅 N3→D4）按此比例折算
 * 兜底倒车距离（actual_forward_cm × 0.30），补偿里程累计偏大。其余门原样倒回。
 * 该值为当前实车标定值；长期应查明为何 actual_forward_cm 比门边实际长度大约 3 倍。 */
#define BLACK_REVERSE_SHORT_SCALE  0.30f

static float black_reverse_distance(float actual_cm, u32 flag)
{
    if ((flag & BLACK_REVERSE_SHORT) != 0u)
        return actual_cm * BLACK_REVERSE_SHORT_SCALE;
    return actual_cm;
}

/* 黑门倒车传感检测：复用现有节点到达规则（MUL2MUL 三线 / MORELED 多灯），
 * 不新增标志位、不改地图边 flag。 */
#define BLACK_REVERSE_CONFIRM_CYCLES 3u
#define BLACK_REVERSE_TIMEOUT_MS     2000u
#define BLACK_REVERSE_MAX_CM         100.0f
#define BLACK_REVERSE_PERIOD_MS      5u
#define BLACK_REVERSE_FORWARD_CORRECT_CM 15.0f  /* 传感倒车后前移补偿：循迹板在车头，检测到路口时轮轴已越过源节点约 15cm */
#define BLACK_REVERSE_LEAVE_CM       8.0f   /* 阶段1：先离开当前黑门检测区至少8cm，才允许武装终点检测 */
#define BLACK_REVERSE_SENSOR_MARGIN_CM 5.0f /* 传感器命中裕量：允许多退一点确保车头板扫到源节点 */
/* 距离兜底时多退的量（车头板前置15cm + 5cm 裕量）：DISTANCE_OK 后退过头，需再前进补偿 */
#define BLACK_REVERSE_FALLBACK_EXTRA_CM \
    (BLACK_REVERSE_FORWARD_CORRECT_CM + BLACK_REVERSE_SENSOR_MARGIN_CM)

static uint8_t black_reverse_detect(u32 detect_flag)
{
    getline_error();
    if ((detect_flag & MUL2MUL) == MUL2MUL && Scaner.lineNum >= 3u)
        return 1u;
    if ((detect_flag & MUL2SING) == MUL2SING && Scaner.lineNum >= 2u && Scaner.ledNum <= 4u)
        return 1u;
    if ((detect_flag & MORELED) == MORELED && Scaner.ledNum >= 5u)
        return 1u;
    return 0u;
}

/* 所有黑门回退统一用 MORELED（多灯检测，比 MUL2MUL 三线更灵敏）。
 * 之前按门对×来向区分（D4→N3 用 MUL2SING、D3→N5 用 MORELED、其余 MUL2MUL），
 * 实车各门退回点灯数都够，统一 MORELED 简化；保留函数便于后续再逐门微调。 */
static u32 black_reverse_detect_flag(uint8_t gate_index, uint8_t dir)
{
    (void)gate_index;
    (void)dir;
    return MORELED;   /* 多灯检测：ledNum >= 5 */
}

/* 黑门倒车结果三态。
 *   SENSOR_OK   — 阶段2传感器连续命中源节点路口，已停车 → 需 15cm 前移补偿
 *   DISTANCE_OK — 未(及)触发传感器但已达 fallback_cm 兜底距离，已停车 → 不做前移补偿
 *   FAILED      — 倒车超时/停锁/超距，已停车 → 不继续二次倒车/盲目转向
 */
typedef enum {
    BLACK_REVERSE_SENSOR_OK = 0,
    BLACK_REVERSE_DISTANCE_OK,
    BLACK_REVERSE_FAILED
} BlackReverseResult_t;

/* Gyro 锁航向倒车：传感器检测与距离兜底合并到同一次连续倒退中，物理上只倒一次车。
 * 带硬超时 / 里程上限 / 停锁保护。
 *
 * 两阶段状态机（防“起点即误判到达”）：
 *   阶段1 LEAVE_CURRENT_MARK — 当前黑门停车点本身可能已满足 detect()(如 ledNum>=5)，
 *       必须先观察到 detect()==0 连续 CONFIRM_CYCLES 个周期、且倒车里程≥LEAVE_CM，
 *       才进入阶段2。
 *   阶段2 FIND_SOURCE_MARK  — 继续倒车，直至 detect()==1 连续 CONFIRM_CYCLES 个周期，
 *       判定退回源节点，立即停车(SENSOR_OK)。
 *
 * 任意时刻倒车里程达到 fallback_cm（>0）→ 停车(DISTANCE_OK)，与传感同一个倒车过程。 */
static BlackReverseResult_t black_reverse_until_flag(u32 detect_flag, float heading,
                                                     float fallback_cm)
{
    TickType_t start = xTaskGetTickCount();
    uint8_t hits = 0u;        /* 阶段2：目标检测区连续命中次数 */
    uint8_t leave_hits = 0u;  /* 阶段1：离开当前检测区连续确认次数 */
    uint8_t phase = 0u;       /* 0=阶段1 LEAVE，1=阶段2 FIND */

    Chassis_ClearMileage();
    Chassis_MotorControl(is_Gyro, -25.0f, -25.0f, heading);

    while (1)
    {
        float mile = fabsf(Chassis_GetMileage());

        /* 1. 两阶段传感检测优先：先找源节点（即使同周期里程也到兜底，也认传感） */
        if (phase == 0u)
        {
            /* 阶段1：离开当前黑门检测区后再武装终点检测 */
            if (!black_reverse_detect(detect_flag) && mile >= BLACK_REVERSE_LEAVE_CM)
            {
                if (++leave_hits >= BLACK_REVERSE_CONFIRM_CYCLES)
                    phase = 1u;
            }
            else
            {
                leave_hits = 0u;
            }
        }
        else
        {
            /* 阶段2：寻找源节点，连续命中即判定到位 */
            if (black_reverse_detect(detect_flag))
            {
                if (++hits >= BLACK_REVERSE_CONFIRM_CYCLES)
                {
                    CarBrake();
                    return BLACK_REVERSE_SENSOR_OK;
                }
            }
            else
            {
                hits = 0u;
            }
        }

        /* 2. 距离兜底：传感没命中且里程已达 fallback_cm → 停车(DISTANCE_OK) */
        if (fallback_cm > 0.0f && mile >= fallback_cm)
        {
            CarBrake();
            return BLACK_REVERSE_DISTANCE_OK;
        }

        /* 3. 最后硬保护：停锁/超时/超距 */
        if (Chassis_IsStopLocked() ||
            (xTaskGetTickCount() - start) >= pdMS_TO_TICKS(BLACK_REVERSE_TIMEOUT_MS) ||
            mile >= BLACK_REVERSE_MAX_CM)
        {
            CarBrake();
            return BLACK_REVERSE_FAILED;
        }
        vTaskDelay(pdMS_TO_TICKS(BLACK_REVERSE_PERIOD_MS));
    }
}

static TrafficRouteStatus_t gate_blocked_swap(uint8_t gate_index, uint8_t dir)
{
    /* 正向阻塞门换线（含门节点）：经 N4 绕行同层另一扇门。 */
    static const uint8_t swap_inner_a[] = {N4, N5, D3, N8, ROUTE_END};   /* N3→N8  黑 → 经 N4 改走 N5→D3→N8 */
    static const uint8_t swap_inner_b[] = {N4, N3, D4, N8, ROUTE_END};   /* N5→N8  黑 → 经 N4 改走 N3→D4→N8 */
    static const uint8_t swap_outer_a[] = {N4, N5, D2, N12, ROUTE_END};  /* N3→N10 黑 → 经 N4 改走 N5→D2→N12 */
    static const uint8_t swap_outer_b[] = {N4, N3, D5, N10, ROUTE_END};  /* N5→N12 黑 → 经 N4 改走 N3→D5→N10 */

    /* 返程黑门换线（用户指定路径，reentry=N4，收敛回返程尾段）：
     * gate3→gate0: N10→N12→D2→N5→N4；gate0→gate3: N12→N8→N10→D5→N3→N4；
     * gate2→gate1: N8→D3→N5→N4；       gate1→gate2: N8→D4→N3→N4。 */
    static const uint8_t ret_swap_g3_g0[] = {N12, D2, N5, N4, ROUTE_END};
    static const uint8_t ret_swap_g0_g3[] = {N8, N10, D5, N3, N4, ROUTE_END};
    static const uint8_t ret_swap_g2_g1[] = {D3, N5, N4, ROUTE_END};
    static const uint8_t ret_swap_g1_g2[] = {D4, N3, N4, ROUTE_END};

    const uint8_t *detour = 0;
    uint8_t reentry = 0u;   /* detour 末节点，用于 Map_SpliceInsertDetour 校验 */
    uint8_t src = 0u;       /* 真实源节点（=lastNode，黑门退回点） */
    uint8_t node_before = 0u;
    float reverse_cm;      /* 预计“轮轴”退回源节点距离（黑门短倒车×0.30折算） */
    float fallback_cm;     /* 距离兜底：reverse_cm + 车头板前置15cm + 裕量5cm */
    float actual_forward_cm;
    BlackReverseResult_t result;
    RouteBuildStatus_t status;

    if (dir == GATE_DIR_FORWARD)
    {
        switch (gate_index)
        {
        case 0u: detour = swap_outer_b; reentry = N10; src = N5; break; /* N5→N12 黑 → 换 N3→D5→N10 */
        case 1u: detour = swap_inner_b; reentry = N8;  src = N5; break; /* N5→N8  黑 → 换 N3→D4→N8  */
        case 2u: detour = swap_inner_a; reentry = N8;  src = N3; break; /* N3→N8  黑 → 换 N5→D3→N8  */
        case 3u: detour = swap_outer_a; reentry = N12; src = N3; break; /* N3→N10 黑 → 换 N5→D2→N12 */
        default:
            return TRAFFIC_ROUTE_STATUS_NO_ROUTE;
        }

        /* 本次正向(出口)换门搜索：当前门不可通 → 标记 blocked。同层换门伴侣(3-gate)
         * 也已被标记 → 同层两扇都无合法出口门 → 安全停车，严禁 D4↔D3 无限换门。 */
        forward_blocked_mask |= (uint8_t)(1u << gate_index);
        if ((forward_blocked_mask & (uint8_t)(1u << (uint8_t)(3u - gate_index))) != 0u)
            return TRAFFIC_ROUTE_STATUS_NO_ROUTE;
    }
    else
    {
        switch (gate_index)
        {
        case 0u: detour = ret_swap_g0_g3; reentry = N4; src = N12; break; /* N12→N5 黑 → 换门5返 */
        case 1u: detour = ret_swap_g1_g2; reentry = N4; src = N8;  break; /* N8→N5  黑 → 换门4返 */
        case 2u: detour = ret_swap_g2_g1; reentry = N4; src = N8;  break; /* N8→N3  黑 → 换门3返 */
        case 3u: detour = ret_swap_g3_g0; reentry = N4; src = N10; break; /* N10→N3 黑 → 换门2返 */
        default:
            return TRAFFIC_ROUTE_STATUS_NO_ROUTE;
        }

        /* 本次返程搜索：当前门不可返程 → 标记 blocked。同层换门伴侣
         * （3-gate：1↔2 即 D3↔D4，0↔3 即 D2↔D5）也已被标记 → 同层两扇都无合法
         * 返程门 → 安全停车，严禁 D4↔D3 无限换门来回。 */
        return_blocked_mask |= (uint8_t)(1u << gate_index);
        if ((return_blocked_mask & (uint8_t)(1u << (uint8_t)(3u - gate_index))) != 0u)
            return TRAFFIC_ROUTE_STATUS_NO_ROUTE;
    }

    /* 换门次数上限防护：同层双阻塞反复换门 + splice 累积失败会死循环。达到上限说明
     * 当前门(方向)确认不可通行 → 安全停车，绝不降级穿门。 */
    if (gate_swap_count[gate_index][dir] >= GATE_SWAP_MAX)
    {
        return TRAFFIC_ROUTE_STATUS_NO_ROUTE;
    }

    /* 重定位来路航向：倒车前先 mpuZreset，把实车 yaw 校到地图帧，
     * 避免长赛程 yaw 漂移（可达 100°+）导致倒车/转向锁错来路航向。 */
    mpuZreset(imu.yaw, nodesr.nowNode.angle);

    /* 退回真实源节点：实际进距用于计算距离兜底（黑门短倒车按 ×0.30 折算）。
     * reverse_cm = 预计“轮轴”回到源节点距离；但车头循迹板要检测到源节点还需额外
     * 前置15cm(+5裕量)。若 fallback_cm 直接用 reverse_cm，距离兜底总在传感器
     * 扫到源节点前提前停车。故 fallback_cm = reverse_cm + FALLBACK_EXTRA_CM。 */
    actual_forward_cm = fabsf(Chassis_GetMileage());
    reverse_cm = black_reverse_distance(actual_forward_cm, nodesr.nowNode.flag);
    fallback_cm = reverse_cm + BLACK_REVERSE_FALLBACK_EXTRA_CM;

    /* 所有黑门统一传感倒车：传感器检测与距离兜底合并到同一次连续倒车（物理上只倒一次车）。
     * 传感中途成功(SENSOR_OK)→前进15cm；距离兜底(DISTANCE_OK)→前进 FALLBACK_EXTRA_CM；
     * 传感失败(FAILED)已真实倒过车，不二次倒车、不盲目转向，降级放行。 */
    result = black_reverse_until_flag(black_reverse_detect_flag(gate_index, dir),
                                      nodesr.nowNode.angle,
                                      fallback_cm);
    if (result == BLACK_REVERSE_SENSOR_OK)
    {
        /* 循迹板在车头：检测到路口时轮轴已越过源节点约 15cm，前移补偿把轮轴拉回源节点，转弯才能落回线上 */
        Chassis_ClearMileage();
        Chassis_DriveDistance_Blocking(is_Gyro, BLACK_REVERSE_FORWARD_CORRECT_CM, 25.0f, nodesr.nowNode.angle);
        CarBrake();
        vTaskDelay(pdMS_TO_TICKS(80));
    }
    else if (result == BLACK_REVERSE_DISTANCE_OK)
    {
        /* 距离兜底停车：此时已多退了(15+5)cm，前进 FALLBACK_EXTRA_CM 把轮轴拉回预计源节点 */
        Chassis_ClearMileage();
        Chassis_DriveDistance_Blocking(is_Gyro, BLACK_REVERSE_FALLBACK_EXTRA_CM, 25.0f, nodesr.nowNode.angle);
        CarBrake();
        vTaskDelay(pdMS_TO_TICKS(80));
    }
    else /* BLACK_REVERSE_FAILED */
    {
        /* 倒车失败（超时/停锁/超距）：已真实倒过车，不知停在何处。绝不盲目穿门或转向，
         * 统一按不可通行门处理 → 安全停车。 */
        return TRAFFIC_ROUTE_STATUS_NO_ROUTE;
    }

    /* 暂存原黑门节点，splice 失败降级放行时恢复，避免脏地图。 */
    node_before = nodesr.nowNode.nodenum;

    nodesr.nowNode.nodenum = src;
    status = Map_SpliceInsertDetour(detour, reentry);
    if (status != ROUTE_BUILD_OK)
    {
        /* 换路拼接失败（异常路线/嵌套换路时 reentry 缺失）：恢复 nowNode、
         * 计入一次失败。这是“已明确不可通行门”的异常 → 绝不降级穿门，安全停车。 */
        nodesr.nowNode.nodenum = node_before;
        if (gate_swap_count[gate_index][dir] < 0xFFu)
            gate_swap_count[gate_index][dir]++;
        return TRAFFIC_ROUTE_STATUS_NO_ROUTE;
    }
    gate_swap_reset(gate_index, dir);   /* 本次换路成功，复位该槽，允许后续正常再换 */

    /* 倒车后原地转向 detour 首段方向，并把地图角度同步为实车 yaw，
     * 使后续 need2turn≈0 直通巡线。同时清掉原门边残留的 nowNode.flag。 */
    CarBrake();
    {
        float target = nodesr.nextNode.angle;
        Chassis_Turn_By_StopGyro_Blocking(target, getAngleZ());
        nodesr.nowNode.angle = target;
    }
    nodesr.nowNode.flag = 0u;

    /* 黑门未通过，不改变扫描朝向（gate_first_passed 维持原值）。 */
    return TRAFFIC_ROUTE_STATUS_OK;
}

/* 正向可通行：把“门→远端”的真实边 prepend 到门路线前再拼接，
 * 保证车真实行驶 Dx→远端(40/45cm) 后才接门路线，不做逻辑瞬移。
 * 正向对端：D2→N12, D3→N8, D4→N8, D5→N10。 */
static RouteBuildStatus_t splice_forward_door_route(uint8_t gate_index,
                                                    const uint8_t *segment)
{
    static const uint8_t forward_far[TRAFFIC_ROUTE_GATE_COUNT] = {N12, N8, N8, N10};
    uint8_t combined[ROUTE_CAPACITY];
    uint8_t i = 0u;
    uint8_t j = 0u;

    if (gate_index >= TRAFFIC_ROUTE_GATE_COUNT || segment == 0)
        return ROUTE_BUILD_INVALID_ARG;

    combined[i++] = forward_far[gate_index];

    while (segment[j] != ROUTE_END)
    {
        if (i >= (uint8_t)(ROUTE_CAPACITY - 1u))
            return ROUTE_BUILD_FULL;
        combined[i++] = segment[j++];
    }
    combined[i] = ROUTE_END;

    return Map_SpliceRemainingRoute(combined);
}
#endif

#ifndef TRAFFIC_ROUTE_UNIT_TEST
TrafficRouteStatus_t TrafficRoute_HandleDoor(void)
{
#if TRAFFIC_ROUTE_VISION_DISABLED
    /* 视觉禁用：纯跑原路线，跳过红绿灯扫描与动态改线。 */
    return TRAFFIC_ROUTE_STATUS_NO_CHANGE;
#else
    VisionResult_t sample;
    VisionDirection_t direction;
    TrafficRouteColor_t effective;
    uint8_t gate_index;
    uint8_t dir = (uint8_t)GATE_DIR_FORWARD;
    const uint8_t *segment;
    RouteBuildStatus_t status;

    gate_index = current_gate(&dir);
    if (gate_index >= TRAFFIC_ROUTE_GATE_COUNT)
        return TRAFFIC_ROUTE_STATUS_NO_CHANGE;   /* 非门区边：直接放行 */

    /* 每次到达门都重新视觉识别：不缓存历史颜色、不因缓存跳过。
     * 扫描侧：第一次实际通过门之前所有门朝右，之后所有门朝左。 */
    direction = gate_first_passed ? VISION_DIRECTION_LEFT
                                  : VISION_DIRECTION_RIGHT;

    if (Vision_ScanSingleSide(direction, &sample) != VISION_STATUS_OK)
    {
        /* 第一次扫描失败：原方向重试一次。两次仍失败 → 不当作已知可通行门，
         * 安全停车，绝不默认穿门。 */
        if (Vision_ScanSingleSide(direction, &sample) != VISION_STATUS_OK)
        {
            last_effective_color = TRAFFIC_ROUTE_COLOR_NONE;
            return TRAFFIC_ROUTE_STATUS_NO_ROUTE;
        }
    }

    effective = TrafficRoute_NormalizeVisionColor(sample.value);
    last_effective_color = (uint8_t)effective;

    switch (effective)
    {
    case TRAFFIC_ROUTE_COLOR_GREEN:
    case TRAFFIC_ROUTE_COLOR_BLUE:
        /* 官方规则：BLACK 禁止 / GREEN 双向 / BLUE 单向(仅FORWARD)。不可通行(RETURN BLUE)
         * → 走阻塞门换线几何（返程受 return_blocked_mask 约束）。 */
        if (!gate_can_pass(gate_index, effective, dir))
            return gate_blocked_swap(gate_index, dir);

        if (dir == GATE_DIR_RETURN)
        {
            /* 合法返程通过：保持当前 route 继续执行，记录首个返程门，并结束本次返程搜索 */
            if (return_gate == TRAFFIC_ROUTE_GATE_INVALID)
            {
                return_gate = gate_index;
                return_blocked_mask = 0u;
            }
            gate_first_passed = 1u;
            return TRAFFIC_ROUTE_STATUS_OK;
        }
        /* 正向可通行（GREEN/BLUE）：第一轮拼接去 P5→P7南极→P8珠峰→回家 的高分路线（按门对选定）。
         * 第二轮不再走这里——都由 ShouldBypassDoor 在 Barrier_Door 顶部直接放行。 */
        segment = RouteCatalog_GetRound1Forward(gate_index);
        if (segment == 0)
            return TRAFFIC_ROUTE_STATUS_NO_ROUTE;

        /* 正向可通行：不瞬移 nowNode，而是把“门→远端”真实边拼到门路线前，
         * 让车先真实走完 Dx→远端 再执行门路线（如 D4→N8→N12→...）。 */
        status = splice_forward_door_route(gate_index, segment);
        if (status != ROUTE_BUILD_OK)
            return TRAFFIC_ROUTE_STATUS_SPLICE_FAILED;

        /* 记录首个正向出口门，并结束本次正向(出口)换门搜索 */
        if (forward_gate == TRAFFIC_ROUTE_GATE_INVALID)
        {
            forward_gate = gate_index;
            forward_blocked_mask = 0u;
        }

        gate_first_passed = 1u;
        return TRAFFIC_ROUTE_STATUS_OK;

    case TRAFFIC_ROUTE_COLOR_BLACK:
        /* 黑门（任何方向）都不可通行：走阻塞门换线几何（返程受 return_blocked_mask 约束）。 */
        return gate_blocked_swap(gate_index, dir);

    case TRAFFIC_ROUTE_COLOR_NONE:
    default:
        /* 无法识别颜色（NONE）：不当作已知可通行门 → 安全停车，绝不默认穿门。 */
        return TRAFFIC_ROUTE_STATUS_NO_ROUTE;
    }
#endif
}
#else
TrafficRouteStatus_t TrafficRoute_HandleDoor(void)
{
    return TRAFFIC_ROUTE_STATUS_SCAN_FAILED;
}
#endif