# 三组复杂岔路统一 Fork Guard Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task with verification checkpoints.

**Goal:** 将 P4→N6→N5、N5→N6→P4、N4→N3→P3 三组复杂岔路统一为局部 `EdgeIgnore=6` 保护，并在保护期间使用完整 16 路 `Cross_Scaner` 做节点到达检测。

**Architecture:** 在 `App/map/map.c` 用一个枚举状态、一个保存的 `EdgeIgnore` 值和三个通用 helper 替换当前 P4 专用 guard。保护更新仍由 `cross_line_update()` 每周期调用；`cross_arrive_check()` 只切换检测器输入指针，不复制或改变原有到达处理逻辑。

**Tech Stack:** STM32 C firmware, existing `Cross_getline()` GPIO scanner, Python `unittest` source-contract tests, CMake/Ninja Release build.

## Global Constraints

- P4→N6→N5：节点前 20cm、节点后 15cm、`EdgeIgnore=6`。
- N5→N6→P4：节点前 20cm、节点后 15cm、`EdgeIgnore=6`。
- N4→N3→P3：节点前 20cm、节点后 20cm、`EdgeIgnore=6`。
- `EdgeIgnore` 只影响循迹控制；保护期间节点检测必须使用完整 16 路 `Cross_Scaner`。
- 不修改地图 flag、angle、step、speed、function、路线长度、节点角度、PID、`Go_Line()`、循迹模式、门、平台、珠峰终点、语音、舵机或既有 P3/N13 专项修复。
- 不使用阻塞 `while`、`Want2Go()` 或 `Chassis_ClearMileage()` 实现保护。
- 不修改 `Sensor/scaner.c`；复用现有无 `EdgeIgnore` 的 `Cross_getline()`。

---

### Task 1: Write failing unified fork-guard contracts

**Files:**
- Modify: `tests/test_p4_n6_fork_guard_contract.py`
- Modify: `tests/test_cross_tracking_contract.py`

**Interfaces:**
- Consumes: current `App/map/map.c` and `Sensor/scaner.c` source text.
- Produces: red tests describing the generic state machine and full-width arrival detector path.

- [ ] **Step 1: Replace P4-specific assertions with the three-route contract**

Update `test_p4_n6_fork_guard_contract.py` to assert:

```python
for macro, value in (
    ("P4_N6_FORK_PRE_CM", "20.0f"),
    ("P4_N6_FORK_POST_CM", "15.0f"),
    ("N5_N6_FORK_PRE_CM", "20.0f"),
    ("N5_N6_FORK_POST_CM", "15.0f"),
    ("N4_N3_FORK_PRE_CM", "20.0f"),
    ("N4_N3_FORK_POST_CM", "20.0f"),
    ("FORK_GUARD_EDGE_IGNORE", "6"),
):
    self.assertRegex(self.source, rf"#define\s+{macro}\s+{re.escape(value)}")

for route in (
    "FORK_GUARD_NONE",
    "FORK_GUARD_P4_N6_N5",
    "FORK_GUARD_N5_N6_P4",
    "FORK_GUARD_N4_N3_P3",
):
    self.assertIn(route, self.source)
```

Assert `fork_guard_enable()` saves `scaner_set.EdgeIgnore`, sets `FORK_GUARD_EDGE_IGNORE`, assigns the route and calls `Line_SetTrackModeBumpless(LEFT_RIGHT_LINE)`. Assert `fork_guard_disable()` restores the saved value, clears `FORK_GUARD_NONE` and performs the same bumpless call.

Assert `cross_fork_guard_update()` contains all six exact node triplets and the matching expressions:

```text
nodesr.nowNode.step - P4_N6_FORK_PRE_CM
mileage >= P4_N6_FORK_POST_CM
nodesr.nowNode.step - N5_N6_FORK_PRE_CM
mileage >= N5_N6_FORK_POST_CM
nodesr.nowNode.step - N4_N3_FORK_PRE_CM
mileage >= N4_N3_FORK_POST_CM
```

Assert it contains no `while`, `Want2Go`, or `Chassis_ClearMileage`.

- [ ] **Step 2: Add the full-width arrival detector contract**

Add assertions that `cross_arrive_check()` contains one shared call using a local scan pointer:

```python
arrive = function_body(self.source, "cross_arrive_check")
self.assertIn("volatile SCANER *arrival_scaner", arrive)
self.assertIn("arrival_scaner = &Scaner", arrive)
self.assertIn("if (fork_guard_route != FORK_GUARD_NONE)", arrive)
self.assertIn("Cross_getline()", arrive)
self.assertIn("arrival_scaner = &Cross_Scaner", arrive)
self.assertIn(
    "arrival_detector_update(arrival_scaner, nodesr.nowNode.flag)",
    arrive,
)
self.assertEqual(arrive.count("arrival_detector_update("), 1)
```

Keep the existing `Cross_getline()` assertion that its implementation contains no `EdgeIgnore`. Update reset and call-order assertions to require `fork_guard_disable()` in `Cross_reset()` and `cross_fork_guard_update()` before `cross_detect_start()` and `cross_arrive_check()`.

- [ ] **Step 3: Verify the tests fail for the expected missing generic behavior**

Run:

```text
python -m unittest tests.test_p4_n6_fork_guard_contract tests.test_cross_tracking_contract -v
```

Expected: failures identify the missing generic enum/helpers, N5/N4 route branches, and `Cross_Scaner` pointer path; no production code has been changed yet.

### Task 2: Implement the generic guard and complete arrival input selection

**Files:**
- Modify: `App/map/map.c:29-31, 368-371, 575-634, 639-645, 891-960, 962-973`

**Interfaces:**
- Consumes: existing `Cross_getline()`, `Cross_Scaner`, `arrival_detector_update()`, and current route state.
- Produces: `ForkGuardRoute_t`, `fork_guard_enable()`, `fork_guard_disable()`, and `cross_fork_guard_update()` used by Cross.

- [ ] **Step 1: Add the shared constants and state**

Replace the P4-only edge-ignore constant and state with:

```c
#define P4_N6_FORK_PRE_CM       20.0f
#define P4_N6_FORK_POST_CM      15.0f
#define N5_N6_FORK_PRE_CM       20.0f
#define N5_N6_FORK_POST_CM      15.0f
#define N4_N3_FORK_PRE_CM       20.0f
#define N4_N3_FORK_POST_CM      20.0f
#define FORK_GUARD_EDGE_IGNORE  6
```

Add the enum and state:

```c
typedef enum
{
    FORK_GUARD_NONE = 0,
    FORK_GUARD_P4_N6_N5,
    FORK_GUARD_N5_N6_P4,
    FORK_GUARD_N4_N3_P3
} ForkGuardRoute_t;

static ForkGuardRoute_t fork_guard_route = FORK_GUARD_NONE;
static int8_t fork_guard_saved_edge_ignore = 0;
```

- [ ] **Step 2: Replace the three P4-only helpers with shared enable/disable/update functions**

Implement `fork_guard_enable(ForkGuardRoute_t route)` exactly with an early return when already active, save-before-set ordering, `FORK_GUARD_EDGE_IGNORE`, route assignment, and bumpless synchronization. Implement `fork_guard_disable()` with an early return when inactive, restore-before-clear ordering, `FORK_GUARD_NONE`, and bumpless synchronization.

Implement `cross_fork_guard_update()` with these exact branches, each ending in `return` after its matching action:

```c
/* P4 -> N6 -> N5: enable at step - P4_N6_FORK_PRE_CM. */
if (nodesr.lastNode.nodenum == P4 &&
    nodesr.nowNode.nodenum  == N6 &&
    nodesr.nextNode.nodenum == N5)
{
    float enable_distance = nodesr.nowNode.step - P4_N6_FORK_PRE_CM;
    if (enable_distance < 0.0f)
        enable_distance = 0.0f;
    if (mileage >= enable_distance)
        fork_guard_enable(FORK_GUARD_P4_N6_N5);
    return;
}

/* N6 -> N5: disable after P4_N6_FORK_POST_CM. */
if (fork_guard_route == FORK_GUARD_P4_N6_N5 &&
    nodesr.lastNode.nodenum == N6 &&
    nodesr.nowNode.nodenum  == N5)
{
    if (mileage >= P4_N6_FORK_POST_CM)
        fork_guard_disable();
    return;
}
```

Add the analogous N5→N6→P4 branches using `FORK_GUARD_N5_N6_P4`, `N5_N6_FORK_PRE_CM`, and `N5_N6_FORK_POST_CM`, then N4→N3→P3 branches using `FORK_GUARD_N4_N3_P3`, `N4_N3_FORK_PRE_CM`, and `N4_N3_FORK_POST_CM`. End with `if (fork_guard_route != FORK_GUARD_NONE) fork_guard_disable();` for abnormal route changes.

- [ ] **Step 3: Use the shared guard at reset and in the normal update order**

Change `Cross_reset()` to call `fork_guard_disable()` before `route_phase_reset()`. In `cross_line_update()`, replace `cross_p4_n6_fork_guard_update()` with `cross_fork_guard_update()` and keep it before `cross_detect_start()` and `cross_arrive_check()`.

- [ ] **Step 4: Select full-width arrival data without duplicating arrival handling**

At the start of `cross_arrive_check()`, declare:

```c
volatile SCANER *arrival_scaner;
```

Keep all existing re-entry, detect-window, temporary-track, arrival confirmation, slowdown, and force-arrival code unchanged. Replace only the sensor read/call with:

```c
arrival_scaner = &Scaner;
getline_error();
if (fork_guard_route != FORK_GUARD_NONE)
{
    Cross_getline();
    arrival_scaner = &Cross_Scaner;
}

if (arrival_detector_update(arrival_scaner, nodesr.nowNode.flag))
{
    /* existing body unchanged */
}
```

- [ ] **Step 5: Run focused tests to verify the implementation passes**

Run:

```text
python -m unittest tests.test_p4_n6_fork_guard_contract tests.test_cross_tracking_contract -v
```

Expected: all tests in both modules pass, including existing P3→N3 and route-detection contracts.

### Task 3: Regression verification and firmware build

**Files:**
- Verify: `App/map/map.c`
- Verify: `Sensor/scaner.c`
- Verify: `tests/test_p4_n6_fork_guard_contract.py`
- Verify: `tests/test_cross_tracking_contract.py`

**Interfaces:**
- Consumes: Task 2 generic guard and arrival source changes.
- Produces: fresh focused regression evidence and a Release firmware artifact; no map or hardware changes.

- [ ] **Step 1: Run related regression tests**

Run:

```text
python -m unittest tests.test_p4_n6_fork_guard_contract tests.test_cross_tracking_contract tests.test_n13_n18_turn_contract tests.test_platform_gesture_contract tests.test_stage_platform_contract tests.test_traffic_light_servo_contract tests.test_voice_mapping_contract -v
```

Expected: all tests relevant to fork guard, Cross flow, prior turn fixes, platform behavior, traffic-light servo protection, and voice mapping pass. Record unrelated existing contract failures separately if encountered.

- [ ] **Step 2: Check the diff and protected files**

Run:

```text
git -c safe.directory="D:/stm32project/游中国探险/26--" -C "D:/stm32project/游中国探险/26--" diff --check
git -c safe.directory="D:/stm32project/游中国探险/26--" -C "D:/stm32project/游中国探险/26--" status --short
```

Confirm production changes are limited to `App/map/map.c`; `App/map/map_message.c`, `Sensor/scaner.c`, and prior specialized logic remain unchanged.

- [ ] **Step 3: Build Release firmware**

Run:

```text
cmake --preset Release
cmake --build --preset Release --parallel 4
```

Expected: exit code 0 and a refreshed `build/Release/explorer_26.elf`; report any existing linker warning separately.

- [ ] **Step 4: Re-run focused fork-guard contracts after build**

Run:

```text
python -m unittest tests.test_p4_n6_fork_guard_contract tests.test_cross_tracking_contract -v
```

Use this fresh output and the build output for the final report. Do not burn or physically tune the robot.
