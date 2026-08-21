from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
MAP_SOURCE = ROOT / "App" / "map" / "map.c"
MAP_MESSAGE_SOURCE = ROOT / "App" / "map" / "map_message.c"


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


def conditional_body(source: str, pattern: str) -> str:
    match = re.search(pattern, source, re.DOTALL)
    if match is None:
        raise AssertionError("missing conditional block")

    start = source.find("{", match.end())
    if start < 0:
        raise AssertionError("conditional has no body")

    depth = 0
    for index in range(start, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[start + 1:index]
    raise AssertionError("unterminated conditional")


class N13N18TurnContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.map_source = MAP_SOURCE.read_text(encoding="utf-8")
        cls.map_message = MAP_MESSAGE_SOURCE.read_text(encoding="utf-8")

    def test_n13_n18_has_a_local_25cm_pre_turn_compensation(self):
        self.assertRegex(
            self.map_source,
            r"#define\s+N13_N18_TURN_FORWARD_CM\s+25\.0f",
        )

        stop_turn = function_body(self.map_source, "cross_stop_turn")
        body = conditional_body(
            stop_turn,
            r"if\s*\(\s*nodesr\.nowNode\.nodenum\s*==\s*N18\s*\)",
        )

        self.assertIn("float pre_turn_cm = 18.0f", body)
        self.assertIn("nodesr.lastNode.nodenum == N13", body)
        self.assertIn("nodesr.nowNode.nodenum == N18", body)
        self.assertIn("nodesr.nextNode.nodenum == B5", body)
        self.assertIn("pre_turn_cm = N13_N18_TURN_FORWARD_CM", body)
        self.assertIn(
            "Chassis_DriveDistance_Blocking(is_Gyro, pre_turn_cm, SPEED1, nodesr.nowNode.angle)",
            re.sub(r"\s+", " ", body),
        )

    def test_other_n18_entries_keep_18cm_and_post_turn_keeps_15cm(self):
        stop_turn = function_body(self.map_source, "cross_stop_turn")
        body = conditional_body(
            stop_turn,
            r"if\s*\(\s*nodesr\.nowNode\.nodenum\s*==\s*N18\s*\)",
        )
        compact = re.sub(r"\s+", "", body)

        self.assertIn("floatpre_turn_cm=18.0f", compact)
        self.assertIn(
            "Chassis_DriveDistance_Blocking(is_Gyro,15.0f,SPEED1,nodesr.nextNode.angle)".replace(" ", ""),
            compact,
        )
        self.assertNotIn("Chassis_DriveDistance_Blocking(is_Gyro,25.0f", compact)

    def test_existing_p3_n3_compensation_is_unchanged(self):
        self.assertRegex(
            self.map_source,
            r"#define\s+P3_N3_TURN_FORWARD_CM\s+25\.0f",
        )
        self.assertRegex(
            self.map_source,
            r"#define\s+P3_N3_POST_TURN_FORWARD_CM\s+8\.0f",
        )
        stop_turn = function_body(self.map_source, "cross_stop_turn")
        self.assertIn("nodesr.lastNode.nodenum == P3", stop_turn)
        self.assertIn("nodesr.nextNode.nodenum == D4", stop_turn)
        self.assertIn("P3_N3_POST_TURN_FORWARD_CM", stop_turn)

    def test_n13_n18_and_n18_b5_map_parameters_are_unchanged(self):
        self.assertRegex(
            self.map_message,
            r"\{N18,\s*CRIGHT\|CLEFT,\s*45,\s*190,\s*SPEED5,\s*NONE\}",
        )
        self.assertRegex(
            self.map_message,
            r"\{B5,\s*RIGHT_LINE\|MORELED,\s*180,\s*33,\s*SPEED25,\s*Hill\}",
        )


if __name__ == "__main__":
    unittest.main()
