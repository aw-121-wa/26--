from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
MAP_SOURCE = ROOT / "App" / "map" / "map.c"
BARRIER_SOURCE = ROOT / "App" / "barrier" / "barrier.c"
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


class NodeSpeedContinuityContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.map_source = MAP_SOURCE.read_text(encoding="utf-8")
        cls.barrier_source = BARRIER_SOURCE.read_text(encoding="utf-8")
        cls.keil_project = KEIL_PROJECT.read_text(encoding="utf-8")

    def test_only_n4_to_n3_and_n3_to_p3_use_gyro_clearance(self):
        clearance = function_body(self.map_source, "cross_need_gyro_clearance")
        update = function_body(self.map_source, "cross_turn_update")

        for token in ("N4", "N3", "P3"):
            self.assertIn(token, clearance)
        self.assertNotIn("nodesr.nextNode.nodenum == N5", clearance)
        self.assertIn("Chassis_DriveDistance_Blocking(is_Gyro, 20.0f", clearance)
        self.assertIn("nodesr.nextNode.speed", clearance)
        self.assertLess(update.index("cross_need_gyro_clearance()"), update.index("cross_node_advance()"))

    def test_node_advance_only_updates_the_new_target_before_line_handoff(self):
        advance = function_body(self.map_source, "cross_node_advance")

        self.assertIn("Chassis_SetTargetSpeed(nodesr.nowNode.speed)", advance)
        self.assertLess(advance.index("Chassis_SetTargetSpeed"), advance.index("Chassis_SetMode(is_Line)"))
        self.assertNotIn("TC_speed.Now", advance)
        self.assertNotIn("TG_speed.Now", advance)

    def test_bridge_entry_and_exit_keep_running_speed_and_pid_history(self):
        bridge = function_body(self.barrier_source, "Barrier_Bridge")

        self.assertNotIn("CarBrake()", bridge)
        self.assertNotIn("motor_pid_clear()", bridge)
        self.assertNotIn("line_pid_obj.integral = 0", bridge)
        self.assertIn("Chassis_MotorControl(is_Gyro", bridge)
        self.assertIn("Line_SetTrackModeBumpless(CENTER_LINE_MODE)", bridge)
        self.assertIn("Chassis_MotorControl(is_Line", bridge)

    def test_keil_project_has_no_side_selector_or_removed_handoff(self):
        self.assertNotIn("..\\Sensor\\line_side_selector.c", self.keil_project)
        self.assertNotIn("motor_handoff.c", self.keil_project)


if __name__ == "__main__":
    unittest.main()
