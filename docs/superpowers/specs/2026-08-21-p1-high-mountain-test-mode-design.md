# P1 到珠峰调头停止场地测试模式设计

## 目标

在 `zt` 分支增加一个由单一编译期开关控制的场地测试模式：机器人从人工放置的 P1 平台顶部出发，复用正式任务的 P1 后半程、平台、门动态改线、南极和珠峰流程，完成珠峰 P8 平台动作、语音、180°调头和摄像头回中后永久停车，不执行珠峰下坡。

## 开关与正式模式

在 `Task/main_task.h` 增加：

```c
#define TEST_P1_TO_HIGH_MOUNTAIN  1
```

该宏同时控制测试起点、测试入口路线和珠峰回中后的停车行为。宏为 `1` 时，测试分支在 `main_task()` 初始化最前面生效；宏为 `0` 时，预处理后保留当前正式初始化、`zhunbei()` 和珠峰下坡流程，不改变正式行为。

## P1 测试初始状态

新增 `mapInit_test_P1_to_HighMountain()` 及其头文件声明。函数只负责重建 Cross 现场，不调用 `Stage()` 或 `zhunbei()`：

- 清零 `map.routetime`、`nodesr.flag`，重置 Cross、底盘保护和显示分数；
- 清空 `route[]`，写入 `B2, N4, N5, N6, P4, N6, N5, N4, N3, P3, N3, D4, N8, ROUTE_END`；
- 用 `Node[getNextConnectNode(N1, P1)]` 作为已完成 P1 的 `lastNode`；
- 用 `Node[getNextConnectNode(P1, N1)]` 作为当前 `nowNode`；
- 用 `Node[getNextConnectNode(N1, B2)]` 作为已预加载的 `nextNode`；
- 设置 `map.point = 1`，使下一次 Cross 推进从 `route[1] = N4` 开始；
- 用 `mpuZreset(imu.yaw, nodesr.nowNode.angle)` 对齐 P1→N1 的正式地图航向，并清零里程。

不手写 P1→N1 或 N1→B2 的角度、速度、步长和标志位，路线连通性由现有 `Node[]` 和 `getNextConnectNode()` 保证。

## 测试起跑入口

`main_task()` 的最高优先级测试分支执行：

1. `Vision_Init()`；
2. `mapInit_test_P1_to_HighMountain()`；
3. 设置底盘停止模式、打开红外挡板检测；
4. 等待检测到挡板，再等待挡板移除；
5. 清零编码器和电机 PID 后进入主 Cross 循环。

该分支不调用 `zhunbei()`。现有 `TEST_START_*` 分支和正式 `Vision_Init()`、`mapInit()`、`zhunbei()` 代码保留在测试宏为 0 的路径中。

## 珠峰停车点

`Barrier_HighMountain()` 保持现有流程直到并包括：挡板检测、珠峰挡板后前进、停车、小人起立/挥手、后退 6cm、P8 语音、180°调头和转向成功检查。转向检查通过后继续执行：

```c
(void)barrier_platform_center();
```

随后在 `#if TEST_P1_TO_HIGH_MOUNTAIN` 下执行 `CarBrake()`、`Chassis_SetMode(is_No)` 和 100ms 周期永久等待，阻止 `high_mountain_descend()`、后续路线和返程逻辑参与测试构建。宏为 0 时该停车代码不存在，原有珠峰下坡逻辑紧接摄像头回中继续执行。

## 范围约束

- 不修改 PID、平台距离、珠峰挡板后前进距离、珠峰后退 6cm、动作顺序、语音编号/时机、摄像头动作、180°算法或坡道参数。
- 不修改 `TrafficRoute`、`RouteCatalog`、南极、P2/P3/P4、桥、楼梯、波浪板和第二轮路线。
- 新增合同测试覆盖宏、P1 初始状态、测试路线连通、起跑分支优先级、珠峰停车位置和 `TEST=0` 正式下坡保留。
