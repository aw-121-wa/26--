from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
MAP_SOURCE = ROOT / "App" / "map" / "map.c"


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


class N8N12StopTurnContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = MAP_SOURCE.read_text(encoding="utf-8")
        cls.stop_turn = function_body(cls.source, "cross_stop_turn")

    def test_n8_n12_uses_local_fifteen_centimeter_stop_turn_distance(self):
        self.assertRegex(
            self.source,
            r"#define\s+N8_N12_TURN_FORWARD_CM\s+15\.0f",
        )
        self.assertRegex(
            self.stop_turn,
            r"nodesr\.lastNode\.nodenum\s*==\s*N8\s*&&\s*"
            r"nodesr\.nowNode\.nodenum\s*==\s*N12",
        )
        self.assertIn("drive_cm = N8_N12_TURN_FORWARD_CM", self.stop_turn)
        self.assertIn("Chassis_DriveDistance_Blocking(is_Gyro, drive_cm, SPEED1, lock_angle)", self.stop_turn)

    def test_other_stop_turn_defaults_remain_unchanged(self):
        self.assertIn("? 0.0f", self.stop_turn)
        self.assertIn("? 15.0f", self.stop_turn)
        self.assertIn(": 18.0f", self.stop_turn)

    def test_n8_n12_force_arrival_exception_matches_215_centimeter_route(self):
        self.assertIn(
            "nodesr.nowNode.nodenum == N12 && nodesr.nowNode.step == 215",
            self.source,
        )
        self.assertNotIn(
            "nodesr.nowNode.nodenum == N12 && nodesr.nowNode.step == 270",
            self.source,
        )


if __name__ == "__main__":
    unittest.main()
