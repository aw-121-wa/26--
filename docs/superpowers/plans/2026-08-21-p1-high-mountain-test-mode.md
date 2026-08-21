# P1 到珠峰调头停止场地测试模式 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 增加由 `TEST_P1_TO_HIGH_MOUNTAIN` 唯一控制的 P1 出发到珠峰 P8 调头停止场地测试模式，同时保持正式任务路线和珠峰下坡行为可由宏关闭恢复。

**Architecture:** 在 `main_task()` 入口用最高优先级预处理分支选择测试起跑或现有正式初始化。新增地图初始化函数只重建 Cross 的 P1→N1→B2 现场和测试路线，后续继续复用现有 `Cross()`、门动态改线和障碍分发。珠峰流程只在摄像头回中后增加同一宏保护的永久停车代码，宏关闭时正式下坡代码保持原位置。

**Tech Stack:** STM32 C firmware, FreeRTOS, CMake/Ninja Release build, Python `unittest` source-contract tests.

## Global Constraints

- `TEST_P1_TO_HIGH_MOUNTAIN` 是本功能唯一总开关，定义在 `Task/main_task.h`，默认值为 `1`。
- 测试模式不得调用 `Stage()` 或 `zhunbei()`，必须调用 `Vision_Init()` 并等待 P1 挡板检测和移除。
- 测试路线固定为 `B2, N4, N5, N6, P4, N6, N5, N4, N3, P3, N3, D4, N8, ROUTE_END`，后续门逻辑继续动态拼接正式高分路线。
- `mapInit_test_P1_to_HighMountain()` 必须通过 `Node[getNextConnectNode(...)]` 复用正式节点属性，不手写角度、速度、步长或标志位。
- 珠峰停止只能插在 `barrier_platform_center()` 之后、`high_mountain_descend()` 之前；必须保留 180°成功检查。
- 不修改 PID、平台/珠峰距离、动作顺序、语音、摄像头动作、180°算法、坡道、TrafficRoute、RouteCatalog 或其它障碍流程。

---

### Task 1: Add the failing P1 test-mode contract

**Files:**
- Create: `tests/test_p1_high_mountain_test_mode_contract.py`
- Read-only references: `Task/main_task.h`, `Task/main_task.c`, `App/map/map.h`, `App/map/map.c`, `App/map/map_message.c`, `App/barrier/barrier.c`

**Interfaces:**
- Consumes: existing source-contract helpers and current `Node[]` map data.
- Produces: assertions for the macro, P1 initial state, test route, startup priority, formal-mode preservation, and high-mountain stop placement.

- [ ] **Step 1: Write the failing test**

Add a `unittest.TestCase` that reads the six source files and asserts:

```python
EXPECTED_ROUTE = (
    "B2, N4, N5, N6, P4, N6, N5, N4, N3, P3, N3, D4, N8, ROUTE_END"
)

def test_unique_test_switch_exists_with_requested_default(self):
    self.assertRegex(self.main_header, r"#define\s+TEST_P1_TO_HIGH_MOUNTAIN\s+1\b")

def test_p1_initializer_preloads_formal_edges_and_route(self):
    body = function_body(self.map_source, "mapInit_test_P1_to_HighMountain")
    self.assertIn("route[i] = ROUTE_END", body)
    self.assertIn(EXPECTED_ROUTE, body.replace("{", "").replace("}", ""))
    self.assertIn("Node[getNextConnectNode(P1, N1)]", body)
    self.assertIn("Node[getNextConnectNode(N1, B2)]", body)
    self.assertIn("map.point = 1", body)
    self.assertIn("mpuZreset(imu.yaw, nodesr.nowNode.angle)", body)

def test_test_startup_has_priority_and_skips_zhunbei(self):
    self.assertLess(
        self.main_source.index("#if TEST_P1_TO_HIGH_MOUNTAIN"),
        self.main_source.index("#if TEST_START_N22_C10"),
    )
    test_branch = preprocessor_branch(self.main_source, "TEST_P1_TO_HIGH_MOUNTAIN")
    for token in ("Vision_Init()", "mapInit_test_P1_to_HighMountain()",
                  "Infrared_ahead == 0", "Infrared_ahead == 1",
                  "encoder_clear()", "motor_pid_clear()"):
        self.assertIn(token, test_branch)
    self.assertNotIn("zhunbei()", test_branch)

def test_formal_branch_keeps_existing_initialization(self):
    formal_branch = preprocessor_else_branch(self.main_source, "TEST_P1_TO_HIGH_MOUNTAIN")
    self.assertIn("Vision_Init()", formal_branch)
    self.assertIn("mapInit()", formal_branch)
    self.assertIn("zhunbei()", formal_branch)

def test_route_edges_are_connected_in_map_data(self):
    for edge in ("N1 -> B2", "B2 -> N4", "N4 -> N5", "N5 -> N6",
                 "N6 -> P4", "P4 -> N6", "N6 -> N5", "N5 -> N4",
                 "N4 -> N3", "N3 -> P3", "P3 -> N3", "N3 -> D4",
                 "D4 -> N8"):
        self.assertIn(edge.replace(" -> ", "→"), self.map_comments)

def test_high_mountain_stops_only_after_center_and_turn_check(self):
    body = function_body(self.barrier_source, "Barrier_HighMountain")
    positions = [body.index(token) for token in (
        "Chassis_Turn_180_Blocking()",
        "fabsf(barrier_angle_normalize(turn_target - getAngleZ()))",
        "barrier_platform_center()",
        "#if TEST_P1_TO_HIGH_MOUNTAIN",
        "high_mountain_descend("
    )]
    self.assertEqual(positions, sorted(positions))
    self.assertIn("Chassis_SetMode(is_No)", body)
    self.assertIn("while (1)", body)

def test_formal_high_mountain_descent_remains_outside_test_guard(self):
    body = function_body(self.barrier_source, "Barrier_HighMountain")
    self.assertIn("#endif", body)
    self.assertIn("if (!high_mountain_descend(turn_target, snapshot.liushui_rate))", body)
```

The route connectivity assertion may use the existing `map_message.c` comments or a small parser for the adjacent `Node[]` entries; it must verify each listed pair against the actual map data rather than merely asserting route text.

- [ ] **Step 2: Run the new test to verify it fails**

Run:

```powershell
python -m unittest tests.test_p1_high_mountain_test_mode_contract -v
```

Expected: FAIL because the macro, initializer, test branch, and guarded stop do not yet exist.

### Task 2: Implement the P1 test map initializer

**Files:**
- Modify: `App/map/map.h:192-210`
- Modify: `App/map/map.c:78-180`

**Interfaces:**
- Consumes: `route[]`, `Node[]`, `getNextConnectNode()`, `Cross_reset()`, `mpuZreset()`, and existing protection/display reset APIs.
- Produces: `void mapInit_test_P1_to_HighMountain(void)` with `route[0] = B2`, `map.point = 1`, `nowNode = Node[getNextConnectNode(P1, N1)]`, and `nextNode = Node[getNextConnectNode(N1, B2)]`.

- [ ] **Step 1: Add the public declaration**

Add immediately after the existing test initializer declarations in `App/map/map.h`:

```c
/**
 * @brief 测试模式地图初始化：从 P1 平台顶部沿 P1→N1 离开并复用正式后半程。
 */
void mapInit_test_P1_to_HighMountain(void);
```

- [ ] **Step 2: Implement the initializer**

Insert a dedicated initializer before the node connection lookup in `App/map/map.c`:

```c
void mapInit_test_P1_to_HighMountain(void)
{
    static const uint8_t test_route[] = {
        B2, N4, N5, N6, P4, N6, N5, N4, N3, P3, N3, D4, N8, ROUTE_END
    };
    uint8_t i;

    map.routetime = 0;
    map.point = 1;
    nodesr.flag = 0;
    Cross_reset();
    Chassis_EnableRollProtection();
    Chassis_EnableYawJumpProtection();
    HmiDisplay_ResetScores();

    for (i = 0; i < ROUTE_CAPACITY; i++)
        route[i] = ROUTE_END;
    for (i = 0; test_route[i] != ROUTE_END; i++)
        route[i] = test_route[i];
    route[i] = ROUTE_END;

    nodesr.lastNode = Node[getNextConnectNode(P1, N1)];
    nodesr.nowNode = Node[getNextConnectNode(P1, N1)];
    nodesr.nextNode = Node[getNextConnectNode(N1, B2)];

    mpuZreset(imu.yaw, nodesr.nowNode.angle);
    Chassis_ClearMileage();
}
```

The implementation must retain the existing formal route and node data. `nodesr.lastNode` may use the same `P1→N1` edge as the simulated prior segment; no manually constructed node attributes are allowed.

- [ ] **Step 3: Run map-focused contracts**

Run:

```powershell
python -m unittest tests.test_p1_high_mountain_test_mode_contract.P1HighMountainTestModeContractTest.test_p1_initializer_preloads_formal_edges_and_route tests.test_p1_high_mountain_test_mode_contract.P1HighMountainTestModeContractTest.test_route_edges_are_connected_in_map_data -v
```

Expected: PASS.

### Task 3: Add the highest-priority test startup branch

**Files:**
- Modify: `Task/main_task.h:9`
- Modify: `Task/main_task.c:18-86`

**Interfaces:**
- Consumes: `TEST_P1_TO_HIGH_MOUNTAIN`, `mapInit_test_P1_to_HighMountain()`, `Vision_Init()`, infrared globals, and existing motor/encoder APIs.
- Produces: a compile-time test branch before all legacy `TEST_START_*` branches; macro-zero path retains the current formal initialization.

- [ ] **Step 1: Add the single switch**

Add beside `LINE_DEBUG_MODE` in `Task/main_task.h`:

```c
#define TEST_P1_TO_HIGH_MOUNTAIN  1  /* 1=P1→珠峰调头停止测试，0=正式任务 */
```

- [ ] **Step 2: Add the branch before legacy test modes**

Make the initialization chain begin with:

```c
#if TEST_P1_TO_HIGH_MOUNTAIN
    Vision_Init();
    mapInit_test_P1_to_HighMountain();

    Chassis_SetMode(is_No);
    motor_all.Lspeed = 0;
    motor_all.Rspeed = 0;
    infrare_open = 1;
    vTaskDelay(100);
    while (Infrared_ahead == 0)
        vTaskDelay(5);
    while (Infrared_ahead == 1)
        vTaskDelay(5);
    encoder_clear();
    motor_pid_clear();
#elif TEST_START_N22_C10
    /* existing legacy test branch unchanged */
```

Keep the existing `#else` block containing `Vision_Init(); mapInit(); zhunbei();` unchanged. Do not add a runtime switch.

- [ ] **Step 3: Run startup contracts**

Run:

```powershell
python -m unittest tests.test_p1_high_mountain_test_mode_contract.P1HighMountainTestModeContractTest.test_unique_test_switch_exists_with_requested_default tests.test_p1_high_mountain_test_mode_contract.P1HighMountainTestModeContractTest.test_test_startup_has_priority_and_skips_zhunbei tests.test_p1_high_mountain_test_mode_contract.P1HighMountainTestModeContractTest.test_formal_branch_keeps_existing_initialization -v
```

Expected: PASS.

### Task 4: Stop after the completed high-mountain turn in test mode

**Files:**
- Modify: `App/barrier/barrier.c:1650-1670`

**Interfaces:**
- Consumes: the existing high-mountain turn-success check and `barrier_platform_center()`.
- Produces: compile-time test stop that prevents the formal descent only when `TEST_P1_TO_HIGH_MOUNTAIN` is enabled.

- [ ] **Step 1: Insert the guarded stop after camera center**

Immediately after the existing center call and before `high_mountain_descend()` add:

```c
#if TEST_P1_TO_HIGH_MOUNTAIN
    /* P1→珠峰专项测试完成：调头并回中后永久停车，不下珠峰。 */
    CarBrake();
    Chassis_SetMode(is_No);
    while (1)
        vTaskDelay(pdMS_TO_TICKS(100));
#endif
```

Do not move or weaken the preceding `Chassis_IsStopLocked()`/angle tolerance check. Do not alter descent or completion code outside the guard.

- [ ] **Step 2: Run high-mountain ordering contracts**

Run:

```powershell
python -m unittest tests.test_p1_high_mountain_test_mode_contract.P1HighMountainTestModeContractTest.test_high_mountain_stops_only_after_center_and_turn_check tests.test_p1_high_mountain_test_mode_contract.P1HighMountainTestModeContractTest.test_formal_high_mountain_descent_remains_outside_test_guard tests.test_polar_mountain_contract -v
```

Expected: PASS.

### Task 5: Run full relevant verification and audit scope

**Files:**
- Verify: `Task/main_task.h`
- Verify: `Task/main_task.c`
- Verify: `App/map/map.h`
- Verify: `App/map/map.c`
- Verify: `App/barrier/barrier.c`
- Verify: `App/map/route_catalog.c`
- Verify: `App/map/traffic_route.c`

- [ ] **Step 1: Run all P1, existing route, obstacle, voice, and zhunbei contracts**

Run:

```powershell
python -m unittest tests.test_p1_high_mountain_test_mode_contract tests.test_platform_gesture_contract.PlatformGestureContractTest.test_platform_forward_distances_are_independent_and_correct tests.test_platform_gesture_contract.PlatformGestureContractTest.test_platforms_use_forward_gesture_reverse_voice_turn_center tests.test_stage_platform_contract tests.test_polar_mountain_contract tests.test_zhunbei_start_contract tests.test_voice_mapping_contract -v
```

Expected: all selected tests pass. Any unrelated pre-existing failures must be reported separately rather than hidden.

- [ ] **Step 2: Confirm no unrelated route or motion changes**

Run:

```powershell
git diff -- App/chassis App/map/route_catalog.c App/map/traffic_route.c App/barrier/barrier.c
git diff --check
```

Expected: only the guarded high-mountain stop appears in `App/barrier/barrier.c`; no PID, distance, route catalog, or traffic-route edits.

- [ ] **Step 3: Configure and build Release firmware**

Run:

```powershell
cmake --preset Release
cmake --build --preset Release --parallel 4
```

Expected: exit code 0 and a refreshed `build/Release/explorer_26.elf`. Record memory usage and any existing linker warnings.

- [ ] **Step 4: Report test-mode behavior and hardware limitation**

Report the changed files, test route, P1 initial heading obtained from the formal node, parking insertion point, behavior for `TEST=1` and `TEST=0`, selected test results, build result, and explicitly state whether hardware flashing was performed.
