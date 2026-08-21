# N8→N12 STOPTURN 转前距离修复设计

## 根因

`N8→N12` 当前地图段为 215cm，节点标志为 `MUL2MUL|STOPTURN`，但
`cross_stop_turn()` 仍走普通 STOPTURN 默认前进 18cm，导致检测到节点后转向位置略晚。
同时，强制到达排除条件仍使用旧的 270cm 参数。

## 目标

只对 `lastNode == N8 && nowNode == N12` 做局部修正：

- 转向前沿当前 140° 航向前进 15cm；
- 保留停车、`DELAY_SHORT` 和原有转向计算/执行流程；
- N8→N12 的强制到达例外使用实际段长 215cm。

## 修改边界

- 在 `App/map/map.c` 增加 `N8_N12_TURN_FORWARD_CM 15.0f`。
- 保留 N20 的 0cm、N18 的 15cm 和其他 STOPTURN 的默认 18cm。
- 只在 `cross_stop_turn()` 中增加 N8→N12 条件覆盖。
- 只把 N8→N12 强制到达例外从 270 改为 215。
- 不修改地图数据、MUL2MUL、STOPTURN、140°角度、其他补偿、Fork Guard、PID、
  循迹、平台、语音、门逻辑和终点逻辑。

## 验证标准

- 契约测试证明 N8→N12 使用 15cm。
- 契约测试证明默认 STOPTURN 仍保留 18cm。
- 契约测试证明强制到达例外使用 215 而不是 270。
- 既有 P3→N3、N13→N18 和 Fork Guard 契约不回归。
- Release 工程编译成功。
