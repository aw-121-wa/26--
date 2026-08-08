from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
MAP_SOURCE = ROOT / "App" / "map" / "map.c"
MOTOR_SOURCE = ROOT / "Task" / "motor_task.c"
KEIL_PROJECT = ROOT / "MDK-ARM" / "explorer_26.uvprojx"


def function_body(source: str, name: str) -> str:
    match = re.search(rf"\b{name}\s*\([^;]*?\)\s*\{{", source, re.DOTALL)
    if match is None:
        raise AssertionError(f"missing function {name}")

    start = match.end() - 1
    depth = 0
    for index in range(start, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[start + 1:index]
    raise AssertionError(f"unterminated function {name}")


class SmoothNodeTransitionContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.map_source = MAP_SOURCE.read_text(encoding="utf-8")
        cls.motor_source = MOTOR_SOURCE.read_text(encoding="utf-8")

    def test_inactive_controller_does_not_destroy_inherited_speed(self):
        update = function_body(self.motor_source, "motor_update_pid_mode")
        self.assertNotIn("motor_all.Cspeed = 0", update)
        self.assertNotIn("motor_all.Gspeed = 0", update)
        self.assertIn("motor_stop_all()", self.motor_source)

    def test_motor_realtime_loop_has_debug_uart_disabled_by_default(self):
        self.assertRegex(
            self.motor_source,
            r"#define\s+MOTOR_TASK_DEBUG_UART\s+0",
        )
        task = function_body(self.motor_source, "motor_task")
        self.assertNotIn("debug_uart_init()", task)
        self.assertNotIn("debug_uart_tick()", task)

    def test_approach_speed_is_applied_before_arrival_detection(self):
        update = function_body(self.map_source, "cross_line_update")
        slowdown = update.find("cross_apply_approach_speed()")
        arrival = update.find("cross_arrive_check()")
        self.assertGreaterEqual(slowdown, 0)
        self.assertGreater(arrival, slowdown)

        arrive_check = function_body(self.map_source, "cross_arrive_check")
        self.assertNotIn("cross_apply_approach_speed", arrive_check)

    def test_direct_node_pass_has_no_brake_or_duplicate_handoff(self):
        turn_update = function_body(self.map_source, "cross_turn_update")
        self.assertNotIn("cross_pass_turn", self.map_source)
        self.assertNotIn("CarBrake()", turn_update)
        self.assertIn("cross_node_advance()", turn_update)

    def test_map_motion_is_bounded_and_turn_failure_is_latched(self):
        self.assertNotIn("Chassis_DriveDistance_Blocking", self.map_source)
        run_turn = function_body(self.map_source, "cross_run_turn")
        self.assertIn("Chassis_DriveDistance_Timeout", run_turn)
        self.assertIn("Chassis_ForceStop(CHASSIS_STOP_MOTION_TIMEOUT)", run_turn)
        self.assertRegex(self.map_source, r"#define\s+MAP_TURN_TIMEOUT_MS\s+1500u")

    def test_track_mode_changes_are_bumpless(self):
        for function in (
            "cross_line_init",
            "cross_track_switch",
            "apply_temp_track_mode",
        ):
            body = function_body(self.map_source, function)
            self.assertIn("Line_SetTrackModeBumpless", body)
            self.assertNotIn("LEFT_RIGHT_LINE =", body)

    def test_keil_project_contains_all_application_sources(self):
        ET.parse(KEIL_PROJECT)
        project = KEIL_PROJECT.read_text(encoding="utf-8")
        for source in (
            r"..\App\map\route_builder.c",
            r"..\App\map\route_catalog.c",
            r"..\App\vision\vision_api.c",
        ):
            self.assertIn(source, project)


if __name__ == "__main__":
    unittest.main()
