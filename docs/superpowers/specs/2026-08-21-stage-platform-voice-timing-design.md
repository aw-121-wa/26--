# 普通平台语音播放时序修复设计

## 背景

`Stage()` 当前在普通平台挡板检测后先执行前进、手势和后退，最后才调用
`barrier_play_platform_voice()`。历史正常提交 `fed34d3` 在挡板检测停车后立即播报，
后续动作再继续，因此当前回归是调用时机变化，而不是语音映射或底层驱动问题。

## 目标

恢复 P1/P3/P4/P5 等普通平台的已验证播放时序：

```text
检测到挡板 → CarBrake() → barrier_play_platform_voice()
→ DELAY_SHORT → mpuZreset() → 既有平台动作
```

`Stage()` 内每次平台流程只播放一次语音。

## 修改边界

- 只修改 `App/barrier/barrier.c` 的 `Stage()`。
- 在挡板检测后的 `CarBrake()` 后调用现有 `barrier_play_platform_voice()`。
- 删除后退稳定等待后的重复调用。
- 保持 `barrier_platform_start_gesture()` 原样。
- 不修改 `Stage_P2()`、`Barrier_SouthPole()`、`Barrier_HighMountain()`、语音映射、
  `Driver/voice_module.c`、串口驱动、路线、PID、循迹和平台距离。

## 验证标准

- `Stage()` 中 `barrier_play_platform_voice()` 只出现一次。
- 唯一调用位于 `while (Infrared_ahead == 0)` 之后且位于 `mpuZreset()` 之前。
- 新契约测试先在修改前失败，修改后通过。
- Release 工程编译成功。
