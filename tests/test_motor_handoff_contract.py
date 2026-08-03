from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "Task" / "motor_task.c"


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


class MotorTransitionContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = SOURCE.read_text(encoding="utf-8")

    def test_line_and_gyro_copy_the_complete_actual_speed_state(self):
        gyro_to_line = function_body(self.source, "mode_inh_g2l")
        line_to_gyro = function_body(self.source, "mode_inh_l2g")

        self.assertIn("TC_speed = TG_speed", gyro_to_line)
        self.assertIn("line_pid_obj = gyroG_pid", gyro_to_line)
        self.assertLess(gyro_to_line.index("TC_speed = TG_speed"), gyro_to_line.index("mode_zero_gyro()"))

        self.assertIn("TG_speed = TC_speed", line_to_gyro)
        self.assertIn("gyroG_pid = line_pid_obj", line_to_gyro)
        self.assertLess(line_to_gyro.index("TG_speed = TC_speed"), line_to_gyro.index("mode_zero_line()"))

    def test_line_first_cycle_continues_gradual_control_without_target_seeding(self):
        mode_update = function_body(self.source, "motor_update_pid_mode")
        targets = function_body(self.source, "motor_update_targets")

        self.assertIn("Go_Line(TC_speed.Now)", mode_update)
        self.assertIn("gradual_cal(&TC_speed, motor_all.Cspeed", mode_update)
        self.assertNotIn("MotorHandoff_SeedLineSpeed", mode_update)
        self.assertNotIn("line_handoff_pending", mode_update)
        self.assertNotIn("line_handoff_pending", targets)

    def test_mode_switch_has_no_external_handoff_patch(self):
        mode_switch = function_body(self.source, "pid_mode_switch")

        self.assertNotIn("line_handoff_pending", self.source)
        self.assertNotIn("MotorHandoff_SeedLineSpeed", self.source)
        self.assertNotIn("motor_handoff.h", self.source)
        self.assertNotIn("line_handoff_pending", mode_switch)

    def test_no_mode_still_performs_a_safe_zero_speed_start(self):
        mode_switch = function_body(self.source, "pid_mode_switch")

        self.assertIn("case is_No", mode_switch)
        self.assertIn("motor_stop_all()", mode_switch)


if __name__ == "__main__":
    unittest.main()
