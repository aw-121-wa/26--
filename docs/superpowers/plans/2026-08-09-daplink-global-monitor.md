# DAPLink Global Monitor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stream live vehicle state through DAPLink COM3 without halting the STM32 core, and record it on the host.

**Architecture:** `Driver/debug_uart.c` emits one bounded `MON` CSV frame per 100 ms using `HAL_UART_Transmit_IT`, so UART transmission cannot block the 5 ms motor task. `tools/daplink_monitor.py` decodes the frame from COM3, prints a concise live status, and appends the full state to CSV.

**Tech Stack:** STM32 HAL/FreeRTOS C11, Python 3 standard library plus pyserial, CMake, unittest.

## Global Constraints

- Reuse USART2 at 230400 baud and DAPLink virtual port COM3.
- Never use GDB polling or breakpoints while the vehicle is moving.
- Emit at most one monitor frame every 100 ms.
- UART transmission is interrupt-driven; when USART2 is busy, drop the current monitor frame.
- Do not alter chassis, route, or obstacle-control decisions.

---

### Task 1: Host frame decoder and COM3 recorder

**Files:**
- Create: `tools/daplink_monitor.py`
- Create: `tests/test_daplink_monitor.py`

**Interfaces:**
- Produces `parse_monitor_line(line: str) -> dict[str, int] | None`, `format_status(frame: dict[str, int]) -> str`, and CLI arguments `--port`, `--baud`, `--csv`.
- Consumes 29-part `MON` frames: the `MON` prefix followed by `<tick>,<round>,<route>,<last>,<current>,<next>,<function>,<mode>,<stop>,<line_detail_hex>,<line_error_x10>,<line_count>,<led_count>,<cross_detail_hex>,<cross_count>,<cross_led>,<yaw_x10>,<pitch_x10>,<roll_x10>,<target_x10>,<gyro_g_x10>,<gyro_t_x10>,<line_pid_x10>,<target_speed_x10>,<actual_speed_x10>,<left_speed_x10>,<right_speed_x10>,<mileage_x10>`.

- [ ] **Step 1: Write the failing test**

```python
from tools.daplink_monitor import format_status, parse_monitor_line

def test_parse_monitor_line_decodes_scaled_and_hex_values():
    frame = parse_monitor_line("MON,100,1,3,7,9,10,2,1,0,00AF,-13,1,2,0010,1,1,1204,23,-5,0,12,-4,8,200,198,201,199,345\\r\\n")
    assert frame["current_node"] == 9
    assert frame["line_detail"] == 0xAF
    assert frame["yaw"] == 120.4
    assert "P3" in format_status(frame)

def test_parse_monitor_line_rejects_invalid_frames():
    assert parse_monitor_line("MON,1,2\\r\\n") is None
    assert parse_monitor_line("not-a-monitor-frame\\r\\n") is None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m unittest tests.test_daplink_monitor`

Expected: FAIL with `ModuleNotFoundError: No module named 'tools.daplink_monitor'`.

- [ ] **Step 3: Write minimal implementation**

Create `tools/daplink_monitor.py`. `parse_monitor_line()` must reject a wrong prefix, wrong field count, non-integer fields, and malformed hexadecimal sensor fields by returning `None`. It converts every `*_x10` field to a float and maps node IDs with the `MapNode` order in `App/map/map.h`. The CLI imports `serial` only after parsing arguments, opens COM3, prints every valid `format_status()` result, writes CSV headers only for an empty file, ignores malformed lines, and reopens after `serial.SerialException`.

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m unittest tests.test_daplink_monitor`

Expected: PASS.

- [ ] **Step 5: Commit**

Run: `git add tools/daplink_monitor.py tests/test_daplink_monitor.py && git commit -m "feat: add DAPLink monitor recorder"`

### Task 2: Non-blocking firmware monitor frame

**Files:**
- Modify: `Driver/debug_uart.c:1-175`
- Modify: `tests/test_debug_snapshot_contract.py`

**Interfaces:**
- Consumes current global state from `nodesr`, `map`, `Scaner`, `Cross_Scaner`, `imu`, `angle`, `line_pid_obj`, `gyroG_pid`, `gyroT_pid`, `motor_all`, `PIDMode`, `Chassis_GetMileage()`, and `Chassis_GetStopReason()`.
- Produces the 28 data fields in Task 1 once per 100 ms maximum.

- [ ] **Step 1: Write the failing test**

```python
def test_debug_uart_emits_nonblocking_complete_monitor_frames(self):
    source = (ROOT / "Driver" / "debug_uart.c").read_text(encoding="utf-8")
    for token in ("#define DEBUG_MONITOR_PERIOD_MS 100u", '"MON,', "HAL_UART_Transmit_IT", "HAL_UART_STATE_READY", "Cross_Scaner.detail", "nodesr.nextNode.nodenum", "Chassis_GetStopReason()"):
        self.assertIn(token, source)
    self.assertNotIn("HAL_UART_Transmit(&huart2", source)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m unittest tests.test_debug_snapshot_contract.DebugSnapshotContractTest.test_debug_uart_emits_nonblocking_complete_monitor_frames`

Expected: FAIL because the monitor period and interrupt-driven transmit are absent.

- [ ] **Step 3: Write minimal implementation**

Replace the selector macros and blocking `dbg_send()` in `Driver/debug_uart.c` with a static 192-byte transmit buffer. `debug_uart_tick()` must compare `xTaskGetTickCount()` against `last_monitor_tick`, return unless 100 ms elapsed, return when `huart2.gState != HAL_UART_STATE_READY`, encode the 29 fields using integer tenths, then call `HAL_UART_Transmit_IT(&huart2, (uint8_t *)monitor_buffer, (uint16_t)frame_length)`. If `snprintf()` overflows or the transmit call is busy/error, do not retry inside the motor task. Keep `debug_uart_init()` as a no-op.

- [ ] **Step 4: Run test to verify it passes**

Run: `python -m unittest tests.test_debug_snapshot_contract.DebugSnapshotContractTest.test_debug_uart_emits_nonblocking_complete_monitor_frames`

Expected: PASS.

- [ ] **Step 5: Commit**

Run: `git add Driver/debug_uart.c tests/test_debug_snapshot_contract.py && git commit -m "feat: stream nonblocking vehicle monitor frames"`

### Task 3: Build and physical serial verification

**Files:**
- Verify: `build/Debug/explorer_26.elf`
- Verify: `logs/daplink-monitor.csv`

**Interfaces:**
- Consumes firmware frames from Task 2 and the host recorder from Task 1.
- Produces a Debug ELF and a CSV whose rows match the frame schema.

- [ ] **Step 1: Run the complete Python suite**

Run: `python -B -m unittest discover -s tests -p "test_*.py"`

Expected: PASS, including the decoder and debug-UART contract.

- [ ] **Step 2: Build Debug firmware**

Run: `cmake --build build/Debug`

Expected: `explorer_26.elf` links without a UART symbol or flash-overflow error.

- [ ] **Step 3: Install host dependency and start recorder**

Run: `python -m pip install pyserial`

Run: `python tools/daplink_monitor.py --port COM3 --baud 230400 --csv logs/daplink-monitor.csv`

Expected: live `P2->N2`-style status lines and a CSV header followed by frames.

- [ ] **Step 4: Confirm non-interference**

Run the vehicle through one normal node transition while the recorder is attached.

Expected: the vehicle does not pause at telemetry intervals; CSV shows changing node, line, and gyro fields.

- [ ] **Step 5: Commit**

Run: `git add docs/superpowers/plans/2026-08-09-daplink-global-monitor.md && git commit -m "docs: add DAPLink monitor implementation plan"`

## Self-Review

- Spec coverage: Tasks 1, 2, and 3 cover COM3 collection, non-blocking telemetry, and software plus physical verification.
- Placeholder scan: no deferred implementation markers remain.
- Type consistency: Task 2 emits exactly the 28 data fields consumed by Task 1 after the `MON` prefix; both use integer tenths for scaled values.
