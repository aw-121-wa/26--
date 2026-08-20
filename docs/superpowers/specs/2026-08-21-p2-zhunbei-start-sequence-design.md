# P2 `zhunbei()` Start Sequence Design

## Goal

在 `zt` 分支中调整 P2 起点的挡板启动时序：挡板移开后才播报准备完毕，随后执行已有的“抬起→动作→放下”动作，动作完成后进入原有 P2 出发流程。

## Scope

- 只修改 `App/barrier/barrier.c` 中的 `zhunbei()`。
- 更新或新增 `tests/test_zhunbei_start_contract.py`，用源码合同锁定时序。
- 不修改 `Stage()`、`barrier_board_detected_action()`、`barrier_play_board_detected_voice()`、平台语音映射或 LSC16 动作宏。
- 不修改 `LINE_DEBUG_MODE` 两个分支中的 P2 出发、下坡、航向、速度、PID 和 pitch 判定逻辑。

## Runtime Sequence

`zhunbei()` 的公共前置流程保持停车、开启红外和短延时。随后执行：

1. 等待 `Infrared_ahead` 从 0 变为 1，表示挡板已检测到。
2. 在 `Infrared_ahead == 1` 期间继续等待，不播报、不执行机械动作。
3. 检测到 `Infrared_ahead` 变为 0 后，调用 `VoiceModule_PlayReadyStart()`。
4. 播报调用之后调用 `Lsc16_RunActionGroupBlocking(LSC16_ACTION_TURN_DONE, LSC16_ACTION_RUN_ONCE, LSC16_WAIT_STAND_MS)`。
5. 动作调用返回后，原样进入 `#if LINE_DEBUG_MODE` 或正常模式的 P2 出发流程。

`LSC16_ACTION_BARRIER_DETECTED` 只从 `zhunbei()` 的启动流程中移除；其宏定义和其他调用继续保留。

## Testing

合同测试从 `void zhunbei(void)` 中提取函数体，并验证：

- 保留挡板检测等待和挡板移除等待；
- 移除等待位于准备语音之前；
- 准备语音位于 `LSC16_ACTION_TURN_DONE` 之前；
- `LSC16_ACTION_BARRIER_DETECTED` 不出现在 `zhunbei()` 函数体中；
- `LSC16_ACTION_TURN_DONE` 位于 P2 出发分支之前；
- `Stage()` 仍保留通用平台的 `barrier_board_detected_action()` 调用。

先运行合同测试确认旧代码因旧时序失败，再进行最小实现，之后运行该测试、完整 Python 测试套件和 CMake Release 构建。

## Acceptance Criteria

- 挡板未检测到或仍存在时，`zhunbei()` 不播报且不执行机械动作。
- 挡板移开后的调用顺序严格为：准备语音→`LSC16_ACTION_TURN_DONE`→P2 出发流程。
- 普通平台流程和已有平台语音编号不发生变化。
- `zt` 分支保持为唯一工作分支，主分支不被修改。
