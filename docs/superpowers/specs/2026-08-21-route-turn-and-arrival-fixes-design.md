# 路线转向与到达兜底修正设计

## 目标

在 `zt` 分支中以最小范围修正四个现场问题：B3→N4→N5 转向前压入距离、N13→N18→B5 转向前压入距离、普通节点里程强制到达阈值，以及 D4 黑门换到 D3 绿灯后 N8→N12→N13 的局部航向重锚定。

## 范围与约束

- 生产代码只修改 `App/map/map.c`。
- 不修改 `App/map/traffic_route.c`、`App/map/route_catalog.c`、地图段长、节点标志、速度、PID、循迹参数、平台、语音、手势和终点逻辑。
- 保持已有 `N8_N12_TURN_FORWARD_CM = 15.0f`、P3→N3 的 25cm+8cm、N18 转后 15cm、普通 STOPTURN 默认 18cm。
- N8→N12、N16→N18、B5→N19 继续排除普通里程强制到达兜底。

## 方案

1. 在 `cross_stop_turn()` 的默认距离初始化后增加 `B3→N4→N5` 的局部 22cm 覆盖；不改变全局 18cm。
2. 将现有 `N13_N18_TURN_FORWARD_CM` 改为 27cm，仅由已有 `N13→N18→B5` 分支消费；其他 N18 入口继续使用 18cm。
3. 将 `ROUTE_FORCE_RATIO` 改为 1.10，并同步更新强制到达注释；保留现有三个例外条件。
4. 在 `N8→N12→N13` 的 STOPTURN 专项中设置局部重锚定标志，在任何锁头直行前执行 `mpuZreset(imu.yaw, nodesr.nowNode.angle)`，然后继续使用现有 15cm和统一转向代码。

## 验证

- 契约测试检查四个参数、三条里程例外、N8→N12→N13 的 `mpuZreset()` 顺序，以及既有 P3→N3/N8→N12/N13→N18 合同。
- 先运行新增测试验证旧代码按预期失败，再运行修复后的相关测试。
- 使用 `git diff --check` 检查补丁格式，并执行 Release CMake 配置和构建。

