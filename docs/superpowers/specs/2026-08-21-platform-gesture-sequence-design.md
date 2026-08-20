# Unified Platform Gesture Sequence Design

## Goal

在 `zt` 分支中同步新的 LSC-16 0~7 动作组定义，并统一 `Stage()`、`Stage_P2()`、`Barrier_SouthPole()`、`Barrier_HighMountain()` 与 `zhunbei()` 的小人动作和平台语音时序。

## Action Mapping

`Driver/lsc16_action.h` 使用以下唯一业务枚举，不保留会映射到旧编号的别名：

| 编号 | 枚举 | 含义 |
| ---: | --- | --- |
| 0 | `LSC16_ACTION_LIE_DOWN` | 机器人躺下 |
| 1 | `LSC16_ACTION_STAND_UP` | 机器人起立 |
| 2 | `LSC16_ACTION_WAVE_LEFT` | 一直挥左手 |
| 3 | `LSC16_ACTION_WAVE_RIGHT` | 一直挥右手 |
| 4 | `LSC16_ACTION_WAVE_STOP` | 停止挥手 |
| 5 | `LSC16_ACTION_CAMERA_LEFT` | 摄像头向左看 |
| 6 | `LSC16_ACTION_CAMERA_RIGHT` | 摄像头向右看 |
| 7 | `LSC16_ACTION_CAMERA_CENTER` | 摄像头回中 |

所有动作等待常量统一按 100ms 处理，LSC-16 通信实现保持不变。

## Shared Helpers

`App/barrier/barrier.c` 增加仅负责机械动作的静态 helper：

- `barrier_platform_start_gesture()`：动作1起立→100ms→动作2左手→100ms→动作3右手。
- `barrier_platform_center()`：动作7摄像头回中→100ms。

helper 不包含车辆移动、语音、转身或坡道逻辑。平台函数继续控制各自的距离、安全检查和失败出口。

平台语音 helper 更名为 `barrier_play_platform_voice()`，调用位置统一放在后退完成并停车之后、180°转身之前。

## Platform Sequences

- `Stage()`：挡板检测→前进 `DISTANCE_PLATFORM_FRONT`→停车→1→2→3→后退 `DISTANCE_PLATFORM_BACK`→停车→平台语音→180°→7→原有下坡。
- `Stage_P2()`：坡顶稳定后等待挡板→前进6cm→停车→1→2→3→后退6cm→停车→P2语音→180°→7→原有流程。
- `Barrier_SouthPole()`：保留8cm前进和5cm后退，改为停车→1→2→3→后退→停车→平台语音→原有转身安全检查→180°→7→原有下坡。
- `Barrier_HighMountain()`：保留8cm前进和6cm后退，改为停车→1→2→3→后退→停车→P8语音→原有转身安全检查→180°→7→原有下坡。

普通平台动作顺序固定为左手在右手之前。动作4不插入平台流程。

## `zhunbei()` Sequence

保留当前已正确的挡板时序：检测到挡板→等待挡板移开→准备完毕语音。语音之后执行：

`STAND_UP`→100ms→`WAVE_LEFT`→100ms→`WAVE_RIGHT`→100ms→`LIE_DOWN`→100ms→`CAMERA_CENTER`→100ms→原有 P2 出发分支。

`LINE_DEBUG_MODE` 两个分支及原有 P2 下坡、航向、速度、PID、pitch 判断保持不变。

## Safety and Scope

- 不修改 `lsc16_action.c` 通信协议。
- 不修改循迹、PID、坡道判定、地图路线、节点、速度、下坡、转向算法或安全保护。
- 对南极和珠峰保留现有失败恢复、停止锁和转身误差检查。
- 为 P2 及独立挡板流程复用带距离、超时和停止锁保护的前方红外等待 helper；不创建第二套通信或运动框架。

## Verification

合同测试覆盖新动作编号、四个平台的动作/语音/转身/回中顺序、`zhunbei()` 的语音和动作顺序、普通平台左手先于右手，以及旧动作别名不再出现在业务代码中。先验证旧代码失败，再运行聚焦测试、全量 Python 测试和 Release CMake 构建。
