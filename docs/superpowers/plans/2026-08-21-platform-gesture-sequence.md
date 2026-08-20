# Unified Platform Gesture Sequence Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 LSC-16 动作组统一为 0~7，并让四类平台及 `zhunbei()` 使用固定的起立→左挥→右挥顺序和正确的平台语音时机。

**Architecture:** 在 `Driver/lsc16_action.h` 中建立唯一的新动作枚举和 100ms 等待常量；在 `App/barrier/barrier.c` 中增加只负责机械动作的两个 helper，各平台保留自己的移动、失败恢复、转向和下坡流程。平台语音 helper 只在后退停车后调用。

**Tech Stack:** STM32 C firmware, FreeRTOS, HAL UART, Python `unittest`, CMake/Ninja.

## Global Constraints

- 只同步 LSC-16 动作组编号和平台/`zhunbei()` 动作语音时序。
- 不修改 LSC-16 通信协议、运动控制、PID、循迹、坡道、地图、节点、速度、下坡、转向算法或安全保护。
- 保留 P2 的原有 `LINE_DEBUG_MODE` 分支和下坡逻辑。
- 保留南极/珠峰失败恢复、停止锁和转身误差检查。
- `zt` 分支不得切换到或修改 `main`。

---

### Task 1: Add failing action and sequence contracts

**Files:**
- Create: `tests/test_platform_gesture_contract.py`
- Modify: `tests/test_zhunbei_start_contract.py`
- Modify: `tests/test_polar_mountain_contract.py`
- Read: `App/barrier/barrier.c`, `Driver/lsc16_action.h`

**Interfaces:**
- Consumes: source text and function bodies from the firmware files.
- Produces: tests for enum values, no old aliases, shared gesture order, platform voice/turn/center order, guarded front-infrared waits, and `zhunbei()` order.

- [ ] **Step 1: Add the new platform contract test**

Implement a `unittest` helper that extracts balanced-brace function bodies. Assert the header contains these exact values:

```python
EXPECTED_ACTIONS = {
    "LSC16_ACTION_LIE_DOWN": 0,
    "LSC16_ACTION_STAND_UP": 1,
    "LSC16_ACTION_WAVE_LEFT": 2,
    "LSC16_ACTION_WAVE_RIGHT": 3,
    "LSC16_ACTION_WAVE_STOP": 4,
    "LSC16_ACTION_CAMERA_LEFT": 5,
    "LSC16_ACTION_CAMERA_RIGHT": 6,
    "LSC16_ACTION_CAMERA_CENTER": 7,
}
```

For `Stage`, `Stage_P2`, `Barrier_SouthPole`, and `Barrier_HighMountain`, assert the body contains and orders `barrier_platform_start_gesture`, its forward-impact operation, the platform voice helper, `Chassis_Turn_180_Blocking`, and `barrier_platform_center`. Assert `barrier_platform_start_gesture` itself orders `STAND_UP < WAVE_LEFT < WAVE_RIGHT`, and the business source contains none of `LSC16_ACTION_BARRIER_DETECTED`, `LSC16_ACTION_TURN_DONE`, or `LSC16_ACTION_STAND_WAVE_LIE_DOWN`.

- [ ] **Step 2: Update `test_zhunbei_start_contract.py`**

Keep the existing barrier-removal-before-ready-voice checks. Replace the old `TURN_DONE` assertion with the required order:

```python
VoiceModule_PlayReadyStart
< LSC16_ACTION_STAND_UP
< LSC16_ACTION_WAVE_LEFT
< LSC16_ACTION_WAVE_RIGHT
< LSC16_ACTION_LIE_DOWN
< LSC16_ACTION_CAMERA_CENTER
< #if LINE_DEBUG_MODE
```

Also assert the `zhunbei()` body contains no old action aliases.

- [ ] **Step 3: Update polar contracts to the guarded infrared helper**

Keep the existing failure and descent assertions, but require `barrier_wait_front_infrared` before `barrier_reverse_distance` in South Pole and High Mountain. Add checks that the helper body contains `barrier_distance_exceeded`, `barrier_wait_expired`, and `Chassis_IsStopLocked`.

- [ ] **Step 4: Run the focused contracts to verify they fail on the old implementation**

Run:

```powershell
python -m unittest discover -s tests -p 'test_platform_gesture_contract.py' -v
python -m unittest discover -s tests -p 'test_zhunbei_start_contract.py' -v
python -m unittest discover -s tests -p 'test_polar_mountain_contract.py' -v
```

Expected: failures for old 0~4 definitions, old aliases, missing shared helpers, old platform action/voice order, and the missing guarded P2/South Pole/High Mountain infrared helper.

### Task 2: Replace action definitions and add shared helpers

**Files:**
- Modify: `Driver/lsc16_action.h`
- Modify: `App/barrier/barrier.c`

**Interfaces:**
- Consumes: existing `Lsc16_RunActionGroupBlocking()` and `Lsc16_RunActionGroup()` APIs.
- Produces: new `Lsc16ActionGroup_t`, `barrier_platform_start_gesture()`, `barrier_platform_center()`, and `barrier_wait_front_infrared()`.

- [ ] **Step 1: Replace the old enum and aliases**

Use explicit values:

```c
typedef enum {
    LSC16_ACTION_LIE_DOWN      = 0u,
    LSC16_ACTION_STAND_UP     = 1u,
    LSC16_ACTION_WAVE_LEFT    = 2u,
    LSC16_ACTION_WAVE_RIGHT   = 3u,
    LSC16_ACTION_WAVE_STOP    = 4u,
    LSC16_ACTION_CAMERA_LEFT  = 5u,
    LSC16_ACTION_CAMERA_RIGHT = 6u,
    LSC16_ACTION_CAMERA_CENTER = 7u
} Lsc16ActionGroup_t;
```

Remove `LSC16_ACTION_INIT_LIE_DOWN`, `LSC16_ACTION_STAND_WAVE_LIE_DOWN`, `LSC16_ACTION_BARRIER_DETECTED`, and `LSC16_ACTION_TURN_DONE`. Keep `LSC16_ACTION_RUN_ONCE`, set `LSC16_WAIT_STAND_MS`, `LSC16_WAIT_LIE_MS`, `LSC16_WAIT_GESTURE_MS`, and `LSC16_WAIT_CAMERA_MS` to `100u`, and retain `LSC16_WAIT_PLATFORM_MS` only if another non-platform flow still needs it.

- [ ] **Step 2: Add the gesture helper**

Implement `barrier_platform_start_gesture()` using blocking transmissions in this exact order:

```c
Lsc16_RunActionGroupBlocking(LSC16_ACTION_STAND_UP,
                             LSC16_ACTION_RUN_ONCE,
                             LSC16_WAIT_STAND_MS);
Lsc16_RunActionGroupBlocking(LSC16_ACTION_WAVE_LEFT,
                             LSC16_ACTION_RUN_ONCE,
                             LSC16_WAIT_GESTURE_MS);
Lsc16_RunActionGroupBlocking(LSC16_ACTION_WAVE_RIGHT,
                             LSC16_ACTION_RUN_ONCE,
                             0u);
```

Return early on a non-`HAL_OK` status. This preserves the requested immediate reverse after the right-hand command while avoiding an asynchronous UART race.

- [ ] **Step 3: Add the camera-center helper**

Implement `barrier_platform_center()` as a blocking `LSC16_ACTION_CAMERA_CENTER` command with `LSC16_WAIT_CAMERA_MS`.

- [ ] **Step 4: Add guarded front-infrared waiting**

Implement `barrier_wait_front_infrared(float max_distance, uint32_t timeout_ms)` using `Infrared_ahead == 0`, `barrier_distance_exceeded(max_distance)`, `barrier_wait_expired(start, timeout_ms)`, `Chassis_IsStopLocked()`, and `vTaskDelay(CONTROL_CYCLE_MS)`. Return `1u` on detection and `0u` on a guard failure.

### Task 3: Reorder platform and `zhunbei()` call chains

**Files:**
- Modify: `App/barrier/barrier.c`

**Interfaces:**
- Consumes: the helpers and new action names from Task 2.
- Produces: four unified platform sequences and the required `zhunbei()` sequence without changing movement/slope logic.

- [ ] **Step 1: Rename and decouple the platform voice helper**

Rename `barrier_play_board_detected_voice()` to `barrier_play_platform_voice()`, remove `barrier_board_detected_action()`, and keep `barrier_platform_voice_index()` unchanged so P1..P8 mapping remains intact.

- [ ] **Step 2: Update `Stage()`**

In `STAGE_TOP`, retain blocker detection and heading setup, then call the existing front-distance move and brake, call `barrier_platform_start_gesture()`, run the existing back-distance move and brake, then call `barrier_play_platform_voice()`. In `STAGE_TURN`, retain `Chassis_Turn_180_Blocking()` and replace `LSC16_ACTION_TURN_DONE` with `barrier_platform_center()` before `STAGE_DESCEND`.

- [ ] **Step 3: Update `Stage_P2()`**

After the existing ramp-top brake/stability delay, call `barrier_wait_front_infrared()` with a bounded distance and timeout, then preserve the existing 6cm front move and heading. Add brake→gesture→6cm reverse→brake→P2 voice→180°→center. Leave PID restoration and `barrier_done(1, 1)` unchanged.

- [ ] **Step 4: Update `Barrier_SouthPole()`**

Replace the immediate old action call with guarded front-infrared waiting, preserve the 8cm front move and 5cm reverse, then use gesture→reverse→brake→platform voice. Preserve `mpuZreset`, turn target calculation, stop/angle checks, and replace the post-turn `TURN_DONE` command with camera center.

- [ ] **Step 5: Update `Barrier_HighMountain()`**

Preserve the two ascent helpers and guarded blocker detection, then keep the 8cm front move. Use gesture before the existing 6cm reverse, move P8 voice after reverse brake, preserve turn safety checks, and replace post-turn `TURN_DONE` with camera center.

- [ ] **Step 6: Update `zhunbei()`**

Keep the existing detection/removal waits and ready voice position. Replace `TURN_DONE` with blocking `STAND_UP`, `WAVE_LEFT`, `WAVE_RIGHT`, `LIE_DOWN`, and `CAMERA_CENTER`, each using 100ms waits, then leave `#if LINE_DEBUG_MODE` and the normal P2 departure body unchanged.

### Task 4: Run regression and build verification

**Files:**
- Verify: `Driver/lsc16_action.h`, `App/barrier/barrier.c`, `App/vision/vision_api.c`
- Verify: all modified tests

- [ ] **Step 1: Run focused action/platform tests**

```powershell
python -m unittest discover -s tests -p 'test_platform_gesture_contract.py' -v
python -m unittest discover -s tests -p 'test_zhunbei_start_contract.py' -v
python -m unittest discover -s tests -p 'test_polar_mountain_contract.py' -v
python -m unittest discover -s tests -p 'test_voice_mapping_contract.py' -v
```

- [ ] **Step 2: Run the full Python suite**

```powershell
python -m unittest discover -s tests -p 'test_*.py'
```

Record exact counts and separate existing unrelated failures from the new contracts.

- [ ] **Step 3: Build Release firmware**

```powershell
cmake --preset Release
cmake --build --preset Release --parallel 4
```

Report exit code, memory usage, warnings, and the generated ELF/BIN artifacts.

- [ ] **Step 4: Audit old aliases and diff scope**

```powershell
rg -n "LSC16_ACTION_(INIT_LIE_DOWN|STAND_WAVE_LIE_DOWN|BARRIER_DETECTED|TURN_DONE)" App Driver --glob '!build/**'
git diff --check
git diff --stat
git status --short --untracked-files=all
```

Expected: no old alias references in business code, no safety or unrelated source changes, and only planned files modified after the documentation commits.
