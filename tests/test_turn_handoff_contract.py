from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
MOTOR_SOURCE = ROOT / "Task" / "motor_task.c"
MAP_SOURCE = ROOT / "App" / "map" / "map.c"
CHASSIS_SOURCE = ROOT / "App" / "chassis" / "chassis_api.c"
CHASSIS_HEADER = ROOT / "App" / "chassis" / "chassis_api.h"


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


class TurnHandoffContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.motor_source = MOTOR_SOURCE.read_text(encoding="utf-8")
        cls.map_source = MAP_SOURCE.read_text(encoding="utf-8")
        cls.chassis_source = CHASSIS_SOURCE.read_text(encoding="utf-8")
        cls.chassis_header = CHASSIS_HEADER.read_text(encoding="utf-8")

    def test_ordinary_turn_uses_original_turn_mode(self):
        run_turn = function_body(self.map_source, "cross_run_turn")

        self.assertIn("Chassis_SetMode(is_Turn)", run_turn)
        self.assertIn("angle.AngleT", run_turn)
        self.assertNotIn("Chassis_Turn_By_Gyro_Blocking", run_turn)

    def test_continuous_gyro_turn_api_is_removed(self):
        self.assertNotIn("Chassis_Turn_By_Gyro_Blocking", self.chassis_header)
        self.assertNotIn("Chassis_Turn_By_Gyro_Blocking", self.chassis_source)

    def test_explicit_stop_turn_keeps_brake_and_wait(self):
        stop_turn = function_body(self.map_source, "cross_stop_turn")

        self.assertIn("CarBrake()", stop_turn)
        self.assertIn("vTaskDelay(DELAY_SHORT)", stop_turn)
        self.assertIn("Chassis_Turn_By_StopGyro_Blocking", stop_turn)

    def test_turn_dispatch_preserves_stop_turn_boundary(self):
        update = function_body(self.map_source, "cross_turn_update")

        self.assertIn("STOPTURN", update)
        self.assertIn("TURN_STOP_ANGLE", update)
        self.assertIn("cross_stop_turn()", update)
        self.assertIn("cross_run_turn();", update)
        self.assertNotIn("if (!cross_run_turn())", update)


if __name__ == "__main__":
    unittest.main()
