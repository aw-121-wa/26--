# P2 `zhunbei()` Start Sequence Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 修改 `zt` 分支 P2 起点 `zhunbei()`，使挡板移开后才播报准备完毕、执行已有站立动作，再进入原有 P2 出发流程。

**Architecture:** 保持现有 `zhunbei()` 单函数结构和公共动作接口，只移动红外等待、准备语音与 `LSC16_ACTION_TURN_DONE` 的相对顺序，并删除该函数内唯一的 `LSC16_ACTION_BARRIER_DETECTED` 调用。用源码合同测试锁定时序，同时验证普通 `Stage()` 仍保留挡板动作调用。

**Tech Stack:** STM32 C firmware, FreeRTOS polling delays, Python `unittest`, CMake/Ninja firmware build.

## Global Constraints

- 只修改 `App/barrier/barrier.c` 的 `zhunbei()` 和新增的 `tests/test_zhunbei_start_contract.py`。
- 保留 `Stage()`、`barrier_board_detected_action()`、`barrier_play_board_detected_voice()`、动作宏和平台语音映射。
- 保留 `LINE_DEBUG_MODE` 两个分支中的 P2 出发、下坡、航向、速度、PID 和 pitch 逻辑。
- `zt` 分支不得切换到或修改 `main` 分支。

---

### Task 1: Add the failing `zhunbei()` sequence contract

**Files:**
- Create: `tests/test_zhunbei_start_contract.py`
- Read: `App/barrier/barrier.c`

**Interfaces:**
- Consumes: source text from `App/barrier/barrier.c`.
- Produces: four source-level assertions for the P2 start sequence and one preservation assertion for `Stage()`.

- [ ] **Step 1: Write the failing test**

Create a `unittest.TestCase` that extracts the text from `void zhunbei(void)` through the `通用平台处理` section marker, then asserts:

```python
class ZhunbeiStartContractTest(unittest.TestCase):
    def test_zhunbei_waits_for_barrier_removal_before_ready_voice(self):
        body = zhunbei_body()
        self.assertIn("while (Infrared_ahead == 0)", body)
        self.assertIn("while (Infrared_ahead == 1)", body)
        self.assertLess(
            body.index("while (Infrared_ahead == 1)"),
            body.index("VoiceModule_PlayReadyStart()"),
        )

    def test_zhunbei_ready_voice_precedes_turn_done_action(self):
        body = zhunbei_body()
        self.assertIn("VoiceModule_PlayReadyStart()", body)
        self.assertIn("LSC16_ACTION_TURN_DONE", body)
        self.assertLess(
            body.index("VoiceModule_PlayReadyStart()"),
            body.index("LSC16_ACTION_TURN_DONE"),
        )

    def test_zhunbei_does_not_use_barrier_detected_action(self):
        self.assertNotIn("LSC16_ACTION_BARRIER_DETECTED", zhunbei_body())

    def test_zhunbei_action_precedes_p2_departure_branches(self):
        body = zhunbei_body()
        action = body.index("LSC16_ACTION_TURN_DONE")
        departure = body.index("#if LINE_DEBUG_MODE")
        self.assertLess(action, departure)

    def test_stage_keeps_generic_barrier_action(self):
        source = BARRIER_SOURCE.read_text(encoding="utf-8")
        stage_start = source.index("void Stage(void)")
        next_section = source.index("/* ========================", stage_start + 1)
        self.assertIn("barrier_board_detected_action(", source[stage_start:next_section])
```

The helper must use the source marker after `zhunbei()` so `LSC16_ACTION_BARRIER_DETECTED` calls in ordinary platform code do not affect the P2-specific assertion.

- [ ] **Step 2: Run the focused test to verify it fails for the old code**

Run:

```powershell
python -m unittest discover -s tests -p 'test_zhunbei_start_contract.py' -v
```

Expected: failure because the current `zhunbei()` contains `LSC16_ACTION_BARRIER_DETECTED` and calls `VoiceModule_PlayReadyStart()` before the `Infrared_ahead == 1` wait.

### Task 2: Apply the minimal P2 sequence change

**Files:**
- Modify: `App/barrier/barrier.c:581-659`

**Interfaces:**
- Consumes: existing `VoiceModule_PlayReadyStart()` and `Lsc16_RunActionGroupBlocking()` APIs.
- Produces: a `zhunbei()` sequence where barrier removal precedes ready voice, ready voice precedes `LSC16_ACTION_TURN_DONE`, and the existing departure branches remain unchanged.

- [ ] **Step 1: Replace only the startup action block**

Keep the detection wait:

```c
while (Infrared_ahead == 0)
    vTaskDelay(5);
```

Immediately follow it with the existing removal wait:

```c
/* 挡板存在期间不播报、不执行机械动作，继续等待挡板移开 */
while (Infrared_ahead == 1)
    vTaskDelay(5);
```

Then place the existing ready voice and `TURN_DONE` action:

```c
/* 挡板确认移开后再播报准备完毕 */
(void)VoiceModule_PlayReadyStart();

/* 播报后执行机器人抬起/动作/放下流程 */
Lsc16_RunActionGroupBlocking(LSC16_ACTION_TURN_DONE,
                             LSC16_ACTION_RUN_ONCE,
                             LSC16_WAIT_STAND_MS);
```

Delete only the `LSC16_ACTION_BARRIER_DETECTED` call from `zhunbei()`.

- [ ] **Step 2: Verify the implementation diff is limited to the planned block**

Run:

```powershell
git diff -- App/barrier/barrier.c
```

Expected: only the startup block changes; the `#if LINE_DEBUG_MODE` branches and all later functions remain unchanged.

### Task 3: Run regression tests and build verification

**Files:**
- Verify: `tests/test_zhunbei_start_contract.py`
- Verify: all `tests/test_*.py`
- Verify: `App/barrier/barrier.c` and `Driver/lsc16_action.h`

**Interfaces:**
- Consumes: the modified source and existing test/build configuration.
- Produces: test evidence, source-order evidence, and a Release firmware artifact.

- [ ] **Step 1: Run the focused contract test**

Run:

```powershell
python -m unittest discover -s tests -p 'test_zhunbei_start_contract.py' -v
```

Expected: all P2 sequence and `Stage()` preservation tests pass.

- [ ] **Step 2: Run the full Python contract suite**

Run:

```powershell
python -m unittest discover -s tests -p 'test_*.py'
```

Record the exact pass/failure/error counts, distinguishing pre-existing unrelated failures from this focused contract.

- [ ] **Step 3: Configure and build the Release preset**

Run:

```powershell
cmake --preset Release
cmake --build --preset Release --parallel 4
```

Expected: exit code 0 and the existing Release ELF/BIN artifacts are produced. Report any linker warnings separately.

- [ ] **Step 4: Audit unchanged safety and platform paths**

Run:

```powershell
rg -n -C 3 "void Stage\(void\)|barrier_board_detected_action|LSC16_ACTION_BARRIER_DETECTED|LSC16_ACTION_TURN_DONE|VoiceModule_PlayReadyStart|#if LINE_DEBUG_MODE|P2_DOWN_BIAS|BEGIN_DOWN" App/barrier/barrier.c Driver/lsc16_action.h
git diff --check
git status --short --untracked-files=all
```

Confirm `LSC16_ACTION_BARRIER_DETECTED` remains defined and used outside `zhunbei()`, `Stage()` remains intact, and only the planned source/test files are modified after the design/plan commits.
