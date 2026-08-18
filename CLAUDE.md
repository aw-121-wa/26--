# explorer_26 工程速查（探险机器人）

> 目的：浓缩整份工程，避免每次对话重复读取大量源文件。本文件是本工程唯一的权威索引。
> 若代码与本文冲突，以代码为准（本文只记录当前理解，可能滞后）。

## 1. 构建环境

| 项 | 值 |
|---|---|
| MCU | STM32F750xx |
| RTOS | FreeRTOS |
| 工具链 | Keil MDK + ARM Compiler V5 (armcc) |
| C 标准 | C99（`<uC99>1`） |
| 源文件编码 | **GBK**（armcc 按 GBK 读取源码） |

**编码坑（重要）**：字符串字面量必须纯 ASCII；UTF-8 中文写进字符串会报 `#870-D`。中文只允许出现在注释里。

## 2. 目录结构

```
App/
  map/         地图 + Cross 状态机（核心）
    map.c         状态机、mapInit、map_function、cross_*、route[]
    map.h         Node/NODESR/Map_State 结构、标志位、枚举、速度宏
    map_message.c Node[126] 边表、Address[]、ConnectionNum[]
  barrier/     障碍物处理（Stage/Bridge/Hill/WavedPlate/SouthPole/...）
    barrier.c   zhunbei() 也在这里
  chassis/     底盘 API（RampCtrl_Blocking、CarBrake、保护开关）
  vision/      视觉（红绿灯/线索/宝藏，串口协议）
Task/          main_task、motor_task、turn、temporary_task（user_init）
Sensor/        imu、bsp_linefollower（红外）、encoder
Core/          HAL/启动
MDK-ARM/       Keil 工程（explorer_26）
```

## 3. 核心数据结构

```c
typedef struct _node {
    u8    nodenum;   // 目标节点编号
    u32   flag;      // 标志位（见 §4）
    float angle;     // 该段方向角（度）
    u16   step;      // 段长(cm)
    float speed;     // 巡线速度
    u8    function;  // 到达该节点后要执行的障碍物类型（§4）
} NODE;

typedef struct _nodesr {
    u8    flag;      // 含 NODE_ARRIVED_FLAG(0x04) = 已到达
    NODE  lastNode;  // 上一节点（其入边数据）
    NODE  nowNode;   // 当前节点（要到达的）
    NODE  nextNode;  // 下一节点（预载）
} NODESR;

struct Map_State {
    uint8_t point;     // route[] 当前消费下标
    uint8_t routetime; // 0=运行中 1=本轮结束 2=停止
};
```

全局：`map`、`nodesr`、`Node[126]`、`ConnectionNum[]`、`Address[]`、`route[100]`、`isAllRoute`、`g_last_arrived_node`。

## 4. 标志位 / 速度 / 障碍物枚举

### 标志位（map.h）
```
NO=1<<0  DLEFT=1<<1  DRIGHT=1<<2  CLEFT=1<<3  CRIGHT=1<<4
MUL2SING=1<<5  MUL2MUL=1<<6  AWHITE=1<<7  RESTMPUZ=1<<8  STOPTURN=1<<9
SLOWDOWN=1<<10  LEFT_LINE=1<<11  RIGHT_LINE=1<<12  MCLEFT=1<<13  MCRIGHT=1<<14
DRIFT=1<<15  MORELED=1<<16  Temp_L=1<<17  Temp_R=1<<18  LiuShui=1<<19
NOTURN=1<<20  Temp_LiuShui=1<<21  L_follow=1<<22  R_follow=1<<23  INGNORE=1<<24
```
- 到达检测相关：DLEFT/DRIGHT/CLEFT/CRIGHT/AWHITE/MORELED/MUL2SING/MUL2MUL（`deal_arrive()` 按这些位匹配传感器）。
- 巡线模式：LEFT_LINE/RIGHT_LINE/LiuShui 决定 `LEFT_RIGHT_LINE`。
- STOPTURN：到点停车原地转。RESTMPUZ：标在边上，**当前无代码消费它**（陀螺仪校准是显式调 `mpuZreset()`，见 §14）。

### 速度（cm/s）
`SPEED0=20 SPEED1=25 SPEED2=30 SPEED3=35 SPEED4=45 SPEED5=55 SPEED25=28`

### 障碍物枚举（barriers）
```
NONE=1  UpStage=2  Bridge=3  Hill=4  LBHill=5  SM=6  View=7  View1=8  BACK=9
BSoutPole=10  QQB=11  BLBS=12  BLBL=13  DOOR=14  BHM=15  IGNORE=16  UNDER=17
Special_node=18  DOOR1=19  UpStageP2=20
```

## 5. 节点枚举（值）

```
S1=0 P1=1 N1=2 B1=3 B2=4 B3=5 N2=6 P2=7 S2=8 P3=9
N3=10 N4=11 N5=12 N6=13 P4=14 N7=15 P5=16 B8=17 B9=18 N8=19
C1=20 C2=21 C3=22 N9=23 N10=24 N12=25 N13=26 P6=27 N14=28 S3=29
S4=30 N15=31 S5=32 C4=33 C5=34 B4=35 B5=36 B6=37 B7=38 N16=39
N18=40 N19=41 P7=42 N20=43 N22=44 C6=45 C7=46 C8=47 C9=48 P8=49
N11=50 C10=51 G1=51   // C10 与 G1 同值，G1 即图纸 C10
```
（注意：无 N17、无 C10 之前缺的编号，枚举就是上面的顺序。）

## 6. 节点连接表（map_message.c）

边表 `Node[126]`，每条边 `{目标, flag, angle, step, speed, function}`。
`Address[node]` = 该节点在 Node[] 中的起始下标；`ConnectionNum[node]` = 邻居个数。

```
Address[53] =
0,1,2,5,7,9,11,14,15,16,17,22,26,30,34,35,38,39,41,43,47,49,51,53,57,63,69,73,74,77,78,79,82,83,85,87,89,91,93,95,98,101,103,104,107,111,113,115,117,118,119,121,124

ConnectionNum[52] =
1,1,3,2,2,2,3,1,1,1,5,4,4,4,1,3,1,2,2,4,2,2,2,4,6,6,4,1,3,1,1,3,1,2,2,2,2,2,2,3,3,2,1,3,4,2,2,2,1,1,2,3
```

完整边表（索引 = Node[] 下标；`-SPEED0` = 负速后退，`SPEED0-7` 是字面表达式=13）：

```
  0 S1→N3   {N3, CLEFT|DLEFT|MUL2MUL, 160, 180, SPEED4, NONE}
  1 P1→N1   {N1, CRIGHT|LEFT_LINE, 180, 30, SPEED2, NONE}
  2 N1→P1   {P1, RIGHT_LINE|MORELED, 0, 30, SPEED2, UpStage}
  3 N1→B2   {B2, LEFT_LINE|MORELED, 142, 30, SPEED2, Hill}
  4 N1→B1   {B1, RESTMPUZ|LEFT_LINE, 180, 25, SPEED1, Bridge}
  5 B1→P2   {P2, LEFT_LINE|CRIGHT|MUL2SING, 180, 33, SPEED1, NONE}
  6 B1→N1   {N1, RIGHT_LINE|MCLEFT|CLEFT|DLEFT|STOPTURN, 0, 5, SPEED2, NONE}
  7 B2→N1   {N1, LEFT_LINE|CRIGHT, -40, 15, SPEED0, NONE}
  8 B2→N4   {N4, CLEFT|MCLEFT|LEFT_LINE, 140, 20, SPEED2, NONE}
  9 B3→N2   {N2, RIGHT_LINE|CLEFT|STOPTURN, -150, 30, SPEED1, NONE}
 10 B3→N4   {N4, CLEFT, 30, 43, SPEED1, NONE}
 11 N2→B3   {B3, NO, 30, 30, SPEED1, BLBS}
 12 N2→P2   {P2, LEFT_LINE, 180, 10, SPEED0, UpStageP2}
 13 N2→B1   {B1, RESTMPUZ|RIGHT_LINE, 0, 20, SPEED0, Bridge}
 14 P2→N2   {N2, RIGHT_LINE, 0, 20, SPEED0, NONE}
 15 S2→N6   {N6, MUL2MUL|RIGHT_LINE|CLEFT|STOPTURN, 45, 100, SPEED4, NONE}
 16 P3→N3   {N3, DRIGHT, 180, 205, SPEED4, NONE}
 17 N3→S1   {S1, NO, -25, 180, SPEED4, View}
 18 N3→P3   {P3, LEFT_LINE, 0, 300, SPEED4, UpStage}
 19 N3→N10  {N10, DLEFT|RIGHT_LINE, 90, 90, SPEED3, DOOR}
 20 N3→N8   {N8, MORELED, 140, 80, SPEED0, DOOR}
 21 N3→N4   {N4, LEFT_LINE|Temp_R|CLEFT, 180, 150, SPEED3, NONE}
 22 N4→B2   {B2, NO, -40, 20, SPEED1, Hill}
 23 N4→N5   {N5, LiuShui|MUL2SING|RIGHT_LINE|Temp_L|NOTURN, 170, 100, SPEED3, NONE}
 24 N4→N3   {N3, DLEFT|Temp_L|LEFT_LINE, 0, 100, SPEED4, NONE}
 25 N4→B3   {B3, LEFT_LINE, -144, 43, SPEED1, BLBS}
 26 N5→N4   {N4, LEFT_LINE|Temp_L|MUL2SING, 0, 120, SPEED4, NONE}
 27 N5→N8   {N8, CLEFT|DLEFT, 35, 80, SPEED0, DOOR}
 28 N5→N12  {N12, AWHITE|RESTMPUZ, 90, 90, SPEED1, DOOR}
 29 N5→N6   {N6, LEFT_LINE|MUL2SING, 180, 104, SPEED4, NONE}
 30 N6→N5   {N5, DLEFT|RIGHT_LINE, 0, 99, SPEED4, NONE}
 31 N6→C1   {C1, CLEFT|DLEFT, 50, 150, SPEED1, NONE}
 32 N6→P4   {P4, LiuShui, 180, 55, SPEED3, UpStage}
 33 N6→S2   {S2, NO, -140, 100, SPEED4, View}
 34 P4→N6   {N6, LEFT_LINE|MUL2SING|NOTURN, 0, 55, SPEED4, NONE}
 35 N7→P5   {P5, NO, 90, 5, SPEED1, UpStage}
 36 N7→B9   {B9, NO, 0, 0, SPEED1, NONE}
 37 N7→B8   {B8, LEFT_LINE|NOTURN, 10, 1, SPEED1, QQB}
 38 P5→N7   {N7, DLEFT|DRIGHT|AWHITE|STOPTURN, -90, 15, SPEED1, NONE}
 39 B8→N7   {N7, NO, 0, 0, SPEED1, NONE}
 40 B8→N9   {N9, LEFT_LINE|MUL2MUL|MUL2SING|STOPTURN, 160, 40, SPEED0-7, NONE}
 41 B9→N7   {N7, DLEFT|MORELED|STOPTURN, 10, 55, SPEED1, NONE}
 42 B9→N9   {N9, NO, 0, 0, SPEED1, NONE}
 43 N8→N3   {N3, CLEFT|LEFT_LINE|MUL2MUL, -45, 60, SPEED0, DOOR}
 44 N8→N10  {N10, MUL2MUL, 33, 140, SPEED3, NONE}
 45 N8→N12  {N12, MUL2MUL, 140, 270, SPEED3, NONE}
 46 N8→N5   {N5, STOPTURN|CLEFT, -140, 150, SPEED0, DOOR}
 47 C1→N6   {N6, CRIGHT, -50, 150, SPEED1, NONE}
 48 C1→C2   {C2, DRIGHT|DLEFT, 125, 30, SPEED1, NONE}
 49 C2→C1   {C1, CLEFT, 170, 154, SPEED1, NONE}
 50 C2→N13  {N13, DRIGHT|DLEFT|CLEFT|CRIGHT|DRIFT, 120, 20, SPEED1, DOOR}
 51 C3→N14  {N14, DLEFT|STOPTURN, 90, 30, SPEED2, NONE}
 52 C3→N9   {N9, RIGHT_LINE|MUL2SING|STOPTURN, 180, 40, SPEED2, NONE}
 53 N9→C3   {C3, DLEFT|CLEFT|STOPTURN|LEFT_LINE, 10, 40, SPEED2, NONE}
 54 N9→N10  {N10, DLEFT|DRIGHT|RIGHT_LINE, 180, 200, SPEED3, NONE}
 55 N9→B8   {B8, NO, 0, 0, SPEED1, QQB}
 56 N9→B9   {B9, LEFT_LINE|NOTURN, -155, 1, SPEED1, QQB}
 57 N10→N9  {N9, LEFT_LINE|CRIGHT|MUL2SING|STOPTURN, 0, 140, SPEED3, NONE}
 58 N10→N15 {N15, DRIGHT|STOPTURN, 90, 20, SPEED2, NONE}
 59 N10→N12 {N12, DRIGHT|RIGHT_LINE, -180, 220, SPEED1, NONE}
 60 N10→N8  {N8, LEFT_LINE|CLEFT|CRIGHT|DLEFT|DRIGHT, -160, 140, SPEED3, NONE}
 61 N10→N3  {N3, DRIGHT|DLEFT, -90, 150, SPEED0, DOOR}
 62 N10→N11 {N11, NO, 180, 80, SPEED0, BLBL}
 63 N12→N11 {N11, NO, 0, 50, SPEED1, BLBL}
 64 N12→N16 {N16, DRIGHT|RIGHT_LINE, 90, 20, SPEED2, NONE}
 65 N12→N13 {N13, CLEFT|CRIGHT|MUL2SING|LEFT_LINE, 180, 70, SPEED3, NONE}
 66 N12→N5  {N5, AWHITE|RIGHT_LINE|RESTMPUZ, -90, 185, SPEED4, NONE}
 67 N12→N8  {N8, CRIGHT|DLEFT, -43, 150, SPEED3, NONE}
 68 N12→P6  {P6, LiuShui, 180, 240, SPEED4, UpStage}
 69 N13→N12 {N12, DLEFT|DRIGHT|LiuShui|RIGHT_LINE, 0, 90, SPEED3, NONE}
 70 N13→N18 {N18, CRIGHT|CLEFT, 45, 190, SPEED3, NONE}
 71 N13→P6  {P6, LiuShui|RIGHT_LINE, 180, 85, SPEED3, UpStage}
 72 N13→C1  {C1, NO, 0, 0, SPEED1, NONE}
 73 P6→N13  {N13, MUL2SING|CLEFT|CRIGHT|LEFT_LINE, 0, 85, SPEED2, NONE}
 74 N14→C3  {C3, DRIGHT|CRIGHT|MORELED, -90, 50, SPEED0, NONE}
 75 N14→C7  {C7, CLEFT|DLEFT, 90, 90, SPEED2, NONE}
 76 N14→S3  {S3, NO, -180, 2, SPEED1, View1}
 77 S3→N14  {N14, INGNORE, 180, 2, -SPEED0, BACK}
 78 S4→N15  {N15, INGNORE, 0, 1, -SPEED0, BACK}
 79 N15→S4  {S4, NO, 0, 1, SPEED1, View1}
 80 N15→C5  {C5, DLEFT, 90, 30, SPEED2, NONE}
 81 N15→N10 {N10, DLEFT|DRIGHT, -90, 20, SPEED2, NONE}
 82 S5→N16  {N16, INGNORE, 0, 1, -SPEED0, BACK}
 83 C4→C8   {C8, MORELED|DRIGHT|CRIGHT, 90, 150, SPEED0, NONE}
 84 C4→N20  {N20, MUL2SING, 155, 180, SPEED4, NONE}
 85 C5→N15  {N15, DLEFT|STOPTURN, -90, 30, SPEED2, NONE}
 86 C5→N18  {N18, DLEFT|CLEFT, 180, 270, SPEED4, NONE}
 87 B4→C5   {C5, DRIGHT, 0, 145, SPEED1, NONE}
 88 B4→N18  {N18, DLEFT|CLEFT|RESTMPUZ, 180, 100, SPEED1, NONE}
 89 B5→N18  {N18, DRIGHT, 0, 70, SPEED3, NONE}
 90 B5→N19  {N19, MORELED, 180, 100, SPEED3, NONE}
 91 B6→N20  {N20, MORELED, 0, 20, SPEED3, NONE}
 92 B6→N22  {N22, DLEFT, 180, 30, SPEED3, NONE}
 93 B7→C9   {C9, MORELED|STOPTURN, 0, 90, SPEED3, NONE}
 94 B7→C6   {C6, DLEFT, 180, 45, SPEED3, NONE}
 95 N16→S5  {S5, NO, 0, 1, SPEED1, View1}
 96 N16→N12 {N12, DLEFT|DRIGHT, -90, 20, SPEED2, NONE}
 97 N16→N18 {N18, DRIGHT|RIGHT_LINE, 90, 25, SPEED2, NONE}
 98 N18→C5  {C5, DRIGHT|CRIGHT, 0, 270, SPEED3, NONE}
 99 N18→B5  {B5, RIGHT_LINE|MORELED, 180, 33, SPEED25, Hill}
100 N18→N16 {N16, DLEFT|CLEFT|STOPTURN, -90, 25, SPEED2, NONE}
101 N19→B5  {B5, NO, 0, 45, SPEED3, Hill}
102 N19→C6  {C6, MORELED|STOPTURN, 90, 150, SPEED2, NONE}
103 P7→N20  {N20, MCLEFT|RIGHT_LINE|STOPTURN, 180, 50, SPEED1, NONE}
104 N20→C4  {C4, MORELED|CLEFT|MCLEFT|STOPTURN, -42, 240, SPEED4, NONE}
105 N20→P7  {P7, LEFT_LINE|RESTMPUZ, 0, 35, SPEED0, BHM}
106 N20→B6  {B6, NO, 180, 30, SPEED1, Hill}
107 N22→C9  {C9, DLEFT, 90, 15, SPEED0, NONE}
108 N22→B6  {B6, RESTMPUZ, 0, 70, SPEED1, Hill}
109 N22→B7  {B7, NO, 180, 115, SPEED25, Hill}
110 N22→C10 {C10, STOPTURN, 180, 15, SPEED2, BLBL}
111 C6→B7   {B7, RESTMPUZ|RIGHT_LINE|MORELED, 0, 130, SPEED25, Hill}
112 C6→N19  {N19, DLEFT, -90, 25, SPEED3, NONE}
113 C7→N14  {N14, DRIGHT|CRIGHT, -90, 190, SPEED3, NONE}
114 C7→C8   {C8, DLEFT|STOPTURN, 180, 100, SPEED2, NONE}
115 C8→C7   {C7, MORELED|STOPTURN|DRIGHT|CRIGHT|RESTMPUZ, 0, 140, SPEED4, NONE}
116 C8→C4   {C4, MCRIGHT|CRIGHT|RESTMPUZ, -90, 0, SPEED1, NONE}
117 C9→N22  {N22, MORELED|AWHITE|STOPTURN, -90, 50, SPEED0, NONE}
118 P8→C10  {C10, DRIGHT, 0, 10, SPEED2, BLBL}
119 N11→N10 {N10, LEFT_LINE|DLEFT, 0, 80, SPEED25, NONE}
120 N11→N12 {N12, RIGHT_LINE|AWHITE|DRIGHT, 180, 50, SPEED25, NONE}
121 C10→C9 {C9, DRIGHT|CRIGHT, 0, 120, SPEED25, NONE}
122 C10→N22 {N22, DLEFT, 0, 40, SPEED3, NONE}
123 C10→P8  {P8, RESTMPUZ, 180, 10, SPEED3, BSoutPole}
```
（Node[124]、Node[125] 是声明为 126 但未初始化的零填充；实际 124 条边。Address[52]=124 即总边数哨兵。）

## 7. 路线 route[]（map.c:56，含下标）

```
 0:N2  1:B1  2:N1  3:P1  4:N1  5:B2  6:N4  7:N5  8:N6  9:P4
10:N6 11:N5 12:N4 13:N3 14:P3 15:N3 16:N8 17:N12 18:N16 19:N18
20:B5 21:N19 22:C6 23:B7 24:C9 25:N22 26:C10 27:P8 28:C10 29:N22
30:B6 31:N20 32:P7 33:N20 34:C4 35:C8 36:C7 37:N14 38:C3 39:N9
40:N10 41:N3 42:N4 43:B3 44:N2 45:P2 46:ROUTE_END(0xFF)
```
完整闭环：P2 起步 → 绕一圈 → 回到 P2 结束（`route[46]=ROUTE_END` 触发本轮结束）。

## 8. Cross 状态机（核心语义，务必先理解）

**三个节点的含义**（与 map.h 注释一致）：
- `nowNode` = 当前**要到达的**节点，其边数据（flag/step/speed/angle/function）描述**当前正在走的这一段**（lastNode→nowNode）。
- `nextNode` = 下一目标（预载的 nowNode→nextNode 边）。
- `lastNode` = 上一节点（其入边）。

**巡线一切用 nowNode**：`cross_line_init`（清零里程+SetTrackMode(nowNode.flag)）、`cross_line_start`（速度=nowNode.speed）、`cross_detect_start`（里程≥0.7×nowNode.step 开始检测）、`cross_arrive_check`（`arrival_detector_update(&Scaner, nowNode.flag)`）、`cross_arrive_slowdown`（nowNode/nextNode 角度）。

**不变式**：正在驶向 `route[k]` 时 → `nowNode=edge(route[k-1]→route[k])`、`nextNode=edge(route[k]→route[k+1])`、`map.point=k+2`。

**节点推进 cross_node_advance()**：
```c
lastNode = nowNode;
nowNode  = nextNode;
nextNode = Node[getNextConnectNode(nowNode.nodenum, route[map.point++])];  // 或 route_last_segment
mpuZreset / 清里程 / SetTargetSpeed(nowNode.speed) / SetMode(is_Line 或 is_Gyro) / 保护开关
```

**每节点流程**（Cross() 周期内）：
1. `cross_line_update`（route_state 0→init→1→start→2，巡线+到达检测）
2. 到达（route_arrived）→ `is_near_end=1`
3. `cross_turn_update`：若 nowNode.function != NONE → 先返回，让 `cross_barrier_update` 执行障碍物；function==NONE → 直接转弯
4. 转弯：STOPTURN 或角度差≥90° → `cross_stop_turn`（停车原地转）；角度差<10° → `cross_pass_turn`（直通）；否则 `cross_run_turn`（行进中转）
5. 障碍物完成后 `barrier_done()` 置 `NODE_ARRIVED_FLAG`(0x04) → `cross_barrier_update` 末尾 `route_phase_reset()` → 回到 `cross_turn_update` 推进
6. `cross_node_advance()` 推进到下一节点

**route_need_turn(ad, ad2)**：ad=当前航向到 nextNode 角度的差，ad2=nowNode.angle 到 nextNode.angle 的差；任一 <10° 视为无需转。STOPTURN 独立判断（两段同角度时 route_need_turn 为 false，不能放 else 分支）。

**强制到达兜底**（cross_arrive_check 末尾）：里程≥`step*1.0` 强制到达，例外——N12(270)、N18(25)、N19(100) 不兜底；P3→N3（N3,step205）与 N3→N8（N8,step80）走满即强制。

## 9. 初始化 + 测试模式

### mapInit()（正常）
```c
map.routetime=0; map.point=0; nodesr.flag=0; Cross_reset();
使能 roll/yaw 保护; HmiDisplay_ResetScores();
nowNode = 手动 P2 {angle=0, function=NONE, speed=SPEED1, step=10, flag=CLEFT|RIGHT_LINE};
nextNode = Node[getNextConnectNode(P2, route[0]=N2)];  // P2→N2
```
**P2 是特例**：nowNode 是手工"平台离场"段（10cm），不是 P2→N2 边。配套特判：
- `route_is_p2_to_n2()`：nowNode==P2 && nextNode==N2 → 居中巡线，50% 后切右循线。
- `cross_special_n2_b1()`：lastNode==P2 && nowNode==N2 && nextNode==B1 → mpuZreset + 陀螺仪直走 N2_B1_PASS_CM + 居中。

### 测试模式 mapInit_test_N22_C10()（map.c，跳过前段）
复用主 route[]，重建中途现场：
```c
map.point = 28;   // route[26]=C10 为当前目标，route[27]=P8 已预载，从 route[28] 继续消费
lastNode = Node[getNextConnectNode(C9, N22)];   // C9→N22 (Node[117])
nowNode  = Node[getNextConnectNode(N22, C10)];  // N22→C10 (Node[110]，STOPTURN+BLBL)
nextNode = Node[getNextConnectNode(C10, P8)];   // C10→P8 (Node[123]，RESTMPUZ+BSoutPole)
```
后续 `P8→C10→N22→B6→N20→P7→N20→C4→C8→C7→N14→C3→N9→N10→N3→N4→B3→N2→P2` 与主路线后半段一致。
**通用测试写法**：中途起点的正确状态 = 用"正在驶向 route[k]"的不变式填 last/now/next + map.point=k+2，而不是照抄 mapInit 的 P2 特例。

### 测试开关（Task/main_task.c）
```
TEST_START_N22_C10  0   // 从 N22 向 C10 出发（已实现）
TEST_START_N22_B6   0   // 未实现 mapInit_test_N22_B6()
TEST_START_P3       0   // 未实现 mapInit_test_P3()
```
三者互斥（#if/#elif/#else）。挡板检测块：`is_No` 停转 → `infrare_open=1` → 等 `Infrared_ahead` 0→1（挡板）→ 1→0（移除）→ `SetTargetSpeed`。

## 10. 障碍物分发 map_function()

```c
UpStage   → Stage()                          return SKIP
Bridge    → Barrier_Bridge()                 (NORMAL)
Hill      → Barrier_Hill()                   (NORMAL)
BLBS      → Barrier_WavedPlate(87.0f)        (NORMAL)
BLBL      → Barrier_WavedPlate(100.0f)       (NORMAL)
DOOR/DOOR1→ Barrier_Door()                   (NORMAL)
BSoutPole → Barrier_SouthPole()              (NORMAL)
BHM       → Barrier_HighMountain()           (NORMAL)
UpStageP2 → Stage_P2()                       return SKIP
default   → 无动作                            NORMAL
```
`MAP_POST_TURN_SKIP`：障碍物内部已处理航向，Cross 只推进不转弯（UpStage/UpStageP2）。
障碍物函数都是阻塞式（`while(1)` 内部状态机），完成后调 `barrier_done()`。

## 11. 底盘 API 与坡道控制

关键函数（chassis_api.h/.c）：
- `Chassis_SetMode(mode)` / `Chassis_SetTargetSpeed(speed)` / `Chassis_MotorControl(mode,l,r,angle)`
- `Chassis_ClearMileage()` / `Chassis_GetMileage()`（cm）
- `CarBrake()`（普通刹车，不锁存）/ `Chassis_ForceStop(reason)`（锁存，`Chassis_IsStopLocked()` 查询）
- `Chassis_DriveDistance_Blocking(mode, dist, speed, angle)`
- `Chassis_Turn_By_StopGyro_Blocking(target, current)` / `Chassis_Turn_180_Blocking()`
- 保护开关：`Chassis_Enable/Disable{AntiSnake, LineLostProtection, RollProtection, YawJumpProtection}`
- 停止锁存原因：`CHASSIS_STOP_*`（LINE_LOST/TIPOVER/YAW_JUMP/STALL/MOTION_TIMEOUT/ROUTE_INVALID/VISION_TIMEOUT/BARRIER_FAILED）

**坡道控制**：
```c
void RampCtrl_Blocking(dir, init_speed, angle,
                       thresh1, speed1, thresh2, speed2,
                       done_thresh, GrayCorrectAngle, max_distance);
```
- 三阶段 pitch 阈值状态机（RAMP_INIT→PHASE1→PHASE2），`while(1)` 阻塞。
- **max_distance>0** 时：累计里程（`fabsf(Chassis_GetMileage())`）超限 → `CarBrake()` + return（里程兜底）。
- 楼梯 `Barrier_Hill()` 用 `HILL_MAX_DISTANCE=120.0f`（barrier.c），上坡/下坡两个 RampCtrl 调用都传它；其余 5 个调用点传 `0, 0.0f`。
- 兜底同时加在 `Barrier_Hill` 外层循环：`if (fabsf(Chassis_GetMileage()) >= HILL_MAX_DISTANCE) { CarBrake(); break; }`。

## 12. 主循环 main_task.c

```c
#if TEST_START_N22_C10 / #elif TEST_START_N22_B6 / #elif TEST_START_P3 / #else
  (测试: mapInit_test_xxx() + 挡板检测)  或  (正常: Vision_Init()+mapInit()+zhunbei())
#endif
#if LINE_DEBUG_MODE
  map.routetime = 2;  // 跳过 Cross 纯巡线调 PID
#endif
encoder_clear();
while(1) {
    if (map.routetime == 0) Cross();
    if (map.routetime == 1) { Chassis_SetMode(is_No); map.routetime = 2; } // 停
    vTaskDelayUntil(&xLastWakeTime, 5/portTICK_RATE_MS);
}
```
`LINE_DEBUG_MODE` 定义在 Task/main_task.h。

## 13. 关键阈值常量（map.c）

```
CONTROL_CYCLE_MS=5  DELAY_SHORT=100  N2_B1_PASS_CM=10
NODE_ARRIVED_FLAG=0x04
ROUTE_HALF_RATIO=0.5  ROUTE_DETECT_RATIO=0.7  ROUTE_SLOW_RATIO=0.7
ROUTE_FORCE_RATIO=1.0  NODE_REENTRY_CM=10  TURN_STOP_ANGLE=90  TURN_NEED_ANGLE=10
```
`get_detect_ratio()`：N16→N18（nowNode==N18 && step==25）用 0.85，其余 0.7。

## 14. 注意事项 / 坑

1. **编码**：字符串字面量纯 ASCII，中文只在注释里（GBK，UTF-8 会 #870-D）。
2. **边界**：只改本地 `C:\Users\orrange\Desktop\26---zt`；zt 远端 clone 和 git 只读，不许动。
3. **岔口处理**：主策略是"归零"（`Go_Line` 层 lineNum>1 || ledNum>3 归零），不是陀螺仪清出。调阈值，不要回头加 `cross_need_gyro_clearance()`（现在恒返回 1 空操作）。
4. **RESTMPUZ 惰性**：该 flag 只写在边上，当前 `.c` 无 `flag & RESTMPUZ` 消费者；陀螺仪校准是显式 `mpuZreset(imu.yaw, nowNode.angle)`（cross_special_n2_b1、zhunbei、Stage、Bridge、SouthPole、HighMountain、user_init）。
5. **障碍物完成后**：靠 `barrier_done()` 置 `NODE_ARRIVED_FLAG` 再回到 `cross_turn_update` 推进；改障碍物时别忘置位。
6. **nowNode 语义**：巡线/到达检测一律读 nowNode（当前段），不是 nextNode。这是最容易搞错的地方。
7. **里程兜底**：`RampCtrl_Blocking` 的 `max_distance` 用累计里程，进 `Barrier_Hill` 前已在别处 `ClearMileage`，所以 120 是整段楼梯的累计上限。
8. **map.point 与测试**：新增测试起点必须按 §8 不变式填 `map.point=k+2`，并正确预载 nextNode，否则第一段/第二段会用错边数据。

## 15. 常用调试线索

- 巡线/到达：`App/map/scaner.h`（Scaner 传感器）、`deal_arrive()`（map.c）。
- 坡道 pitch：`imu`（`getAngleZ()` 带 compensateZ 补偿；`mpuZreset(sensor, refer)` 写基准）。
- 串口调试输出：cross_stop_turn / cross_run_turn 会 `HAL_UART_Transmit(&huart2, ...)` 打 `T:... A:...`。
- 显示：`HmiDisplay_*`（hmi_display.c）。

## 16. 坡道障碍判定与调试（2026-08 更新）

### 俯仰角判定（barrier.c:79-84）
- 符号：**pitch 正 = 抬头（上坡），负 = 低头（下坡）**。`basic_p` 基准俯仰角运行时测（每次跑不同，观测 0.59/1.38）。
- 阈值：`BEGIN_UP = AFTER_UP = basic_p+5`，`UP_PITCH = basic_p+20`，`BEGIN_DOWN = AFTER_DOWN = basic_p-5`，`DOWN_PITCH = basic_p-20`。
- 判定模式：**上坡 = pitch 先 >= BEGIN_UP 再 <= AFTER_UP（先升后降）；下坡 = pitch 先 <= BEGIN_DOWN 再 >= AFTER_DOWN（先降后升）**。
- **原则**：坡道位置判定一律以 pitch 为主；循迹灯只用于平地起始寻线+最终恢复；里程只做防卡死兜底（250cm）。2026-08 已把 `high_mountain_second_ascend`、`high_mountain_descend` 中段、谷底过渡、`south_pole_ascend` 四处从 ledNum 判定改为纯 pitch 判定。

### STOP:8 语义
- `STOP:%d mile:%.1f step:%d`（debug_uart.c:82）：`mile`=自上次 ClearMileage 累计 cm；`step`=`g_barrier_step`。
- `STOP:8`=`CHASSIS_STOP_BARRIER_FAILED`。`stop_lock_set` 锁存**第一个**非 NONE 原因，故 STOP:8 = 不是保护(倾覆/丢线)先触发，而是障碍物里程/超时兜底先触发。
- `g_barrier_step` 只南极设置（SP_STEP_*=1..10）；珠峰不设置，BHM 停车时 `step:` 是南极遗留旧值（10=SP_STEP_DONE），不能定位 BHM 段。

### 调试打印
- debug_uart.c `DBG_RAMP=1`：`RAMP dir= state= pitch= basic=`。`g_ramp_dir/state` 只在 RampCtrl_Blocking(楼梯)设、退出不重置，BHM/南极跑时 RAMP 行的 dir/state 是遗留无意义值，**只看 pitch 列**。
- barrier.c 宏（barrier.c:46-49）：`HM_ASCEND_LOG=1`（`[HM] ascend2`）、`SP_ASCEND_LOG`、`HILL_APPROACH_LOG`、`BARRIER_DEBUG_LOG`。
- `imu.pitch` 在 USART3_IRQHandler(imu.c:73) 中断更新；`debug_uart_tick` 在 motor_task，阻塞障碍物 while(1) 期间也能打印。

### 已定位根因
- **Hill 楼梯出不去**：`RAMP_DETECT_HILL` 15→8（接近段翻过坡顶导致 ASCEND 在坡顶后触发，卡 RAMP_INIT 负 pitch）。
- **BHM 珠峰下坡 STOP:8 mile:120（已修）**：`high_mountain_descend` 谷底段 `barrier_wait_pitch_above(AFTER_DOWN, 120)`（barrier.c:1587）120cm 兜底太小，下坡回平需 >120cm，`return 0` → barrier_fail → 不调 barrier_done → function 不切换。已把 barrier.c:1585/1587 两处 120→250（注意共用 line 1582 的 ClearMileage 计数器）。若仍卡在后面（`below(BEGIN_DOWN,150)`/`above(AFTER_DOWN,180)` 第三段下坡），说明珠峰有第三段下坡，再放大。
