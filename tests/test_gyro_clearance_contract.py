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


class GyroClearanceContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = MAP_SOURCE.read_text(encoding="utf-8")

    def test_only_n4_n3_and_n3_p3_run_20cm_gyro_clearance(self):
        clearance = function_body(self.source, "cross_need_gyro_clearance")

        self.assertIn("nodesr.nowNode.nodenum == N4", clearance)
        self.assertIn("nodesr.nextNode.nodenum == N3", clearance)
        self.assertIn("nodesr.nowNode.nodenum == N3", clearance)
        self.assertIn("nodesr.nextNode.nodenum == P3", clearance)
        self.assertNotIn("nodesr.nextNode.nodenum == N5", clearance)
        self.assertIn("Chassis_DriveDistance_Blocking(is_Gyro, 20.0f", clearance)
        self.assertIn("nodesr.nextNode.speed", clearance)
        self.assertIn("Chassis_IsStopLocked()", clearance)

    def test_clearance_runs_before_line_handoff_and_node_advance(self):
        update = function_body(self.source, "cross_turn_update")

        self.assertIn("if (!route_need_turn(ad, ad2))", update)
        self.assertIn("cross_need_gyro_clearance()", update)
        self.assertLess(
            update.index("cross_need_gyro_clearance()"),
            update.index("cross_pass_turn()"),
        )
        self.assertLess(
            update.index("cross_need_gyro_clearance()"),
            update.index("cross_node_advance()"),
        )


if __name__ == "__main__":
    unittest.main()
