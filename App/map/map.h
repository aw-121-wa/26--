#ifndef __MAP_H
#define __MAP_H

#include "sys.h"

/* ======================== 节点标志位定义 ======================== */

#define NO          (1<<0)      /* 无特殊标志 */
#define DLEFT       (1<<1)      /* 左边横线（到达检测：左半边5灯亮） */
#define DRIGHT      (1<<2)      /* 右边横线（到达检测：右半边5灯亮） */
#define CLEFT       (1<<3)      /* 左岔路（到达检测：左斜线45度） */
#define CRIGHT      (1<<4)      /* 右岔路（到达检测：右斜线45度） */
#define MUL2SING    (1<<5)      /* 多条变一条 */
#define MUL2MUL     (1<<6)      /* 多条变多条 */
#define AWHITE      (1<<7)      /* 全黑/全白检测 */
#define RESTMPUZ    (1<<8)      /* 陀螺仪校准 */
#define STOPTURN    (1<<9)      /* 停车转弯 */
#define SLOWDOWN    (1<<10)     /* 减速 */
#define LEFT_LINE   (1<<11)     /* 左循线模式 */
#define RIGHT_LINE  (1<<12)     /* 右循线模式 */
#define MCLEFT      (1<<13)     /* 中间偏左 */
#define MCRIGHT     (1<<14)     /* 中间偏右 */
#define DRIFT       (1<<15)     /* 陀螺仪转弯 */
#define MORELED     (1<<16)     /* 更多灯亮 */
#define Temp_L      (1<<17)     /* 临时切换左循线 */
#define Temp_R      (1<<18)     /* 临时切换右循线 */
#define LiuShui     (1<<19)     /* 流水灯模式 */
#define NOTURN      (1<<20)     /* 不转弯 */
#define Temp_LiuShui (1<<21)    /* 临时切换流水灯 */
#define L_follow    (1<<22)     /* 左循线加强转弯 */
#define R_follow    (1<<23)     /* 右循线加强转弯 */
#define INGNORE     (1<<24)     /* 忽略（后退） */

/* ======================== 路线结束标记 ======================== */

#define ROUTE_END   0xFF

/* ======================== 节点枚举 ======================== */

enum MapNode {
    S1 = 0,     /* 起点/终点标志 */
    P1 = 1,     /* 平台1 */
    N1 = 2,     /* 普通节点1 */
    B1 = 3,     /* 桥 */
    B2 = 4,     /* 坡 */
    B3 = 5,     /* 翘板桥 */
    N2 = 6,     /* 普通节点2 */
    P2 = 7,     /* 平台2 */
    S2 = 8,     /* 特殊节点2 */
    P3 = 9,     /* 平台3 */
    N3 = 10,
    N4 = 11,
    N5 = 12,
    N6 = 13,
    P4 = 14,
    N7 = 15,
    P5 = 16,
    B8 = 17,
    B9 = 18,
    N8 = 19,
    C1 = 20,
    C2 = 21,
    C3 = 22,
    N9 = 23,
    N10 = 24,
    N12 = 25,
    N13 = 26,
    P6 = 27,
    N14 = 28,
    S3 = 29,
    S4 = 30,
    N15 = 31,
    S5 = 32,
    C4 = 33,
    C5 = 34,
    B4 = 35,
    B5 = 36,
    B6 = 37,
    B7 = 38,
    N16 = 39,
    N18 = 40,
    N19 = 41,
    P7 = 42,
    N20 = 43,
    N22 = 44,
    C6 = 45,
    C7 = 46,
    C8 = 47,
    C9 = 48,
    P8 = 49,
    N11 = 50,
    G1 = 51
};

/* ======================== 障碍物类型枚举 ======================== */

enum barriers {
    NONE = 1,           /* 无障碍 */
    UpStage = 2,        /* 上台阶 */
    Bridge = 3,         /* 过桥 */
    Hill = 4,           /* 楼梯/山地 */
    LBHill = 5,         /* 左坡 */
    SM = 6,             /* 假山 */
    View = 7,           /* 景点（转弯） */
    View1 = 8,          /* 景点（直行） */
    BACK = 9,           /* 后退 */
    BSoutPole = 10,     /* 南极 */
    QQB = 11,           /* 跷跷板 */
    BLBS = 12,          /* 短波浪板 */
    BLBL = 13,          /* 长波浪板 */
    DOOR = 14,          /* 门 */
    BHM = 15,           /* 高山 */
    IGNORE = 16,        /* 忽略 */
    UNDER = 17,         /* 下方 */
    Special_node = 18,  /* 特殊节点 */
    DOOR1 = 19,         /* 门1 */
    UpStageP2 = 20      /* P2平台台阶 */
};

/* ======================== 速度定义 ======================== */

#define SPEED0      20
#define SPEED1      25
#define SPEED2      30
#define SPEED3      35
#define SPEED4      45
#define SPEED5      55
#define SPEED25     28

/* ======================== 节点结构体 ======================== */

typedef struct _node {
    u8 nodenum;     /* 节点编号 */
    u32 flag;       /* 节点标志位 */
    float angle;    /* 角度 */
    u16 step;       /* 线长(cm) */
    float speed;    /* 寻线速度 */
    u8 function;    /* 节点功能（障碍物类型） */
} NODE;

/* ======================== 节点状态结构体 ======================== */

typedef struct _nodesr {
    u8 flag;            /* 状态标志位 */
    NODE lastNode;      /* 上一个节点 */
    NODE nowNode;       /* 当前节点（要到达的） */
    NODE nextNode;      /* 下一个节点 */
} NODESR;

/* ======================== 障碍后转向处理 ======================== */

typedef enum {
    MAP_POST_TURN_NORMAL = 0,   /* 障碍结束后由 Cross 执行通用转向 */
    MAP_POST_TURN_SKIP          /* 障碍内部已处理航向，Cross 只推进节点 */
} MapPostTurnAction_t;

/* ======================== 地图状态结构体 ======================== */

struct Map_State {
    uint8_t point;      /* 路径数组当前索引 */
    uint8_t routetime;  /* 地图运行次数 */
};

/* ======================== 全局变量声明 ======================== */

extern struct Map_State map;
extern NODESR nodesr;
extern NODE Node[];
extern uint8_t ConnectionNum[];
extern uint8_t Address[];
extern u8 route[];
extern uint8_t isAllRoute;

/* ======================== 函数声明 ======================== */

/**
 * @brief  地图初始化（第一轮）
 */
void mapInit(void);

/**
 * @brief  地图初始化（第二轮）
 */
void mapInit1(void);

/**
 * @brief  获取从当前节点到目标节点的连接在Node数组中的下标
 * @param  nownode  当前节点编号
 * @param  nextnode 目标节点编号
 * @return uint8_t  Node数组下标
 */
u8 getNextConnectNode(u8 nownode, u8 nextnode);

/**
 * @brief  Cross 状态机 - 节点间处理核心
 */
void Cross(void);

/**
 * @brief  重置 Cross 状态机内部状态（由 mapInit 调用，确保第二轮从干净状态开始）
 */
void Cross_reset(void);

/**
 * @brief  障碍物功能分发
 * @param  fun 功能编号
 */
MapPostTurnAction_t map_function(u8 fun);

/**
 * @brief  路口到达判断
 * @param  scaner   循迹器数据
 * @param  node_flag 节点标志位
 * @return uint8_t  1=到达, 0=未到达
 */
uint8_t deal_arrive(volatile void *scaner, u32 node_flag);

/* ======================== 障碍物函数声明 ======================== */

void zhunbei(void);
void Stage_P2(void);
void Barrier_Bridge(void);
void Barrier_Hill(void);
void Barrier_WavedPlate(float length);

#endif /* __MAP_H */
