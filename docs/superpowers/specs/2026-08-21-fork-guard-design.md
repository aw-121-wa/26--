# 三组复杂岔路统一 Fork Guard 设计

## 目标

在不修改地图参数、循迹模式、PID、`Go_Line()` 或既有专项修复的前提下，统一保护以下三条路线的复杂节点区域：

- P4 → N6 → N5：节点前 20cm、节点后 15cm。
- N5 → N6 → P4：节点前 20cm、节点后 15cm。
- N4 → N3 → P3：节点前 20cm、节点后 20cm。

三组保护均只将 `scaner_set.EdgeIgnore` 设为 6，使循迹控制使用中间 4 路；通过节点后距离后恢复进入保护前保存的设置。

## 现状与范围

当前 `App/map/map.c` 只有 P4→N6 专用状态、开启/关闭函数和更新函数。此次用一个通用状态替换这套专用实现，并保持 `cross_line_update()` 中“先更新保护、再启动检测、再进行到达检测”的时序。

本次只修改：

- `App/map/map.c`
- 受影响的 Python 契约测试

地图数据、`Sensor/scaner.c` 的 `Cross_getline()` 不修改。现有 `Cross_getline()` 已直接读取完整 16 路 GPIO 并写入 `Cross_Scaner`，适合作为保护期间的节点检测源。

## 统一状态接口

在 `map.c` 中定义：

```c
typedef enum
{
    FORK_GUARD_NONE = 0,
    FORK_GUARD_P4_N6_N5,
    FORK_GUARD_N5_N6_P4,
    FORK_GUARD_N4_N3_P3
} ForkGuardRoute_t;
```

统一保存：

- 当前保护路线状态 `fork_guard_route`。
- 开启前的 `fork_guard_saved_edge_ignore`。

`fork_guard_enable()` 只在无保护状态时保存并设置 `EdgeIgnore=6`，然后调用 `Line_SetTrackModeBumpless(LEFT_RIGHT_LINE)`；`fork_guard_disable()` 恢复原值、清空状态并再次无扰同步。异常离开路线、`Cross_reset()` 或测试模式切换都通过统一关闭函数清理。

## 动态更新

`cross_fork_guard_update()` 每个正常循迹周期读取当前段里程：

- 当 `lastNode/nowNode/nextNode` 精确匹配三组路线时，在 `nowNode.step - PRE_CM` 处开启。
- 当保护状态与已通过节点后的 `lastNode/nowNode` 匹配时，在 `POST_CM` 处关闭。
- 其他节点组合下若仍有保护状态，立即恢复保存的 `EdgeIgnore`。

更新函数不阻塞、不清零里程、不调用 `Want2Go()`，也不改变 `LEFT_RIGHT_LINE` 的地图要求。

## 到达检测数据流

`cross_arrive_check()` 保留现有所有门控、临时循线清出、确认计数、强制到达和到达后的处理逻辑。仅把检测器输入抽象为局部指针：

```c
volatile SCANER *arrival_scaner = &Scaner;

getline_error();
if (fork_guard_route != FORK_GUARD_NONE)
{
    Cross_getline();
    arrival_scaner = &Cross_Scaner;
}

if (arrival_detector_update(arrival_scaner, nodesr.nowNode.flag))
{
    /* 原有处理保持不变 */
}
```

这样 `getline_error()` 仍受 `EdgeIgnore=6` 影响并服务于循迹控制，而保护期间的 `MUL2SING`、`DLEFT` 等节点标志由完整 16 路 `Cross_Scaner` 检测，不会被边缘屏蔽削弱。

## 不变量

- N5→N6、N6→P4、N4→N3、N3→P3 的地图 flag、角度、step、速度、function 不变。
- P3→N3→D4 的 25cm+8cm 转向补偿不变。
- N13→N18 的 25cm 转前补偿不变。
- 门、平台、珠峰终点、语音和舵机逻辑不变。
- 保护开启时只能影响循迹控制的 `EdgeIgnore`，不能改变节点识别传感器源。

## 验证

新增/更新契约测试检查：

- 三组常量和精确节点匹配。
- 开启保存原值、设置 6、无扰切换；关闭恢复原值、清空状态、无扰切换。
- 更新函数先于检测函数调用，且不包含阻塞行驶逻辑。
- `Cross_reset()` 使用统一关闭函数。
- `cross_arrive_check()` 保护期间调用 `Cross_getline()` 并将 `Cross_Scaner` 传给同一个到达处理逻辑。
- 既有 P4→N6、P3→N3、N13→N18 和地图数据保持不变。

最后运行相关契约测试、`git diff --check` 和 Release CMake 构建；不执行烧录和实车调参。
