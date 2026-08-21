# 普通平台语音时序修复实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** 恢复普通 `Stage()` 在挡板检测停车后立即播放平台语音的时序，并确保流程只播放一次。

**Architecture:** 保留现有 `barrier_play_platform_voice()` 和平台动作流程，只移动 `Stage()` 内唯一的语音调用点。通过源码契约测试锁定调用顺序和单次调用，避免影响 P2、南极、珠峰及底层语音模块。

**Tech Stack:** STM32 C firmware, CMake, Python `unittest` contract tests.

## Global Constraints

- 只修改 `App/barrier/barrier.c` 的 `Stage()` 和 `tests/test_stage_platform_contract.py` 的契约测试。
- 不修改手势 helper、`Stage_P2()`、南极、珠峰、语音映射或底层语音驱动。
- 不增加语音重发、ACK、DMA、中断发送或额外 3 秒等待。

---

### Task 1: 增加失败契约测试

**Files:**
- Modify: `tests/test_stage_platform_contract.py`
- Test: `tests/test_stage_platform_contract.py`

- [ ] **Step 1: 写入 Stage 语音调用顺序断言**

从 `void Stage(void)` 截取到 `void Stage_P2(void)` 之前的函数体，断言：

```python
self.assertEqual(stage_body.count("barrier_play_platform_voice()"), 1)
self.assertLess(stage_body.index("while (Infrared_ahead == 0)"),
                stage_body.index("barrier_play_platform_voice()"))
self.assertLess(stage_body.index("barrier_play_platform_voice()"),
                stage_body.index("mpuZreset("))
self.assertLess(stage_body.index("barrier_play_platform_voice()"),
                stage_body.index("barrier_platform_start_gesture()"))
```

- [ ] **Step 2: 运行测试确认当前代码失败**

Run:

```text
python -m unittest tests.test_stage_platform_contract -v
```

Expected: 新增测试因语音调用仍位于后退流程之后而失败。

### Task 2: 最小修复 Stage 语音时序

**Files:**
- Modify: `App/barrier/barrier.c:789-807`

- [ ] **Step 1: 在挡板检测停车后恢复语音调用**

将 `Stage()` 的平台顶部流程调整为：

```c
CarBrake();
barrier_play_platform_voice();
vTaskDelay(DELAY_SHORT);
mpuZreset(...);
```

- [ ] **Step 2: 删除后退后的重复播放**

保留后退、刹车和稳定等待，只删除其后的 `barrier_play_platform_voice()`。

- [ ] **Step 3: 不改动相邻平台流程**

确认 `barrier_platform_start_gesture()` 调用序列、P2、南极、珠峰和语音 helper 均未改变。

### Task 3: 回归和构建验证

**Files:**
- Verify: `App/barrier/barrier.c`
- Verify: `tests/test_stage_platform_contract.py`

- [ ] **Step 1: 运行定向测试**

```text
python -m unittest tests.test_stage_platform_contract tests.test_voice_mapping_contract -v
```

- [ ] **Step 2: 检查差异范围**

```text
git diff --check
git diff --name-only
```

确认生产代码只有 `App/barrier/barrier.c`，且没有驱动、P2、南极、珠峰或映射文件变更。

- [ ] **Step 3: 编译 Release 工程**

```text
cmake --preset Release
cmake --build --preset Release --parallel 4
```
