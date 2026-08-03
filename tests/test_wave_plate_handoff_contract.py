from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
BARRIER_SOURCE = ROOT / "App" / "barrier" / "barrier.c"
MAP_SOURCE = ROOT / "App" / "map" / "map.c"
SCANER_SOURCE = ROOT / "Sensor" / "scaner.c"
SCANER_HEADER = ROOT / "Sensor" / "scaner.h"


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


def assert_in_order(test_case: unittest.TestCase, body: str, *tokens: str) -> None:
    positions = [body.index(token) for token in tokens]
    test_case.assertEqual(
        positions,
        sorted(positions),
        "tokens are not in the required handoff order",
    )


class WavePlateHandoffContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.barrier = BARRIER_SOURCE.read_text(encoding="utf-8")
        cls.map = MAP_SOURCE.read_text(encoding="utf-8")
        cls.scaner = SCANER_SOURCE.read_text(encoding="utf-8")
        cls.scaner_header = SCANER_HEADER.read_text(encoding="utf-8")

    def test_wave_plate_keeps_center_mode_for_map_handoff(self):
        wave_body = function_body(self.barrier, "Barrier_WavedPlate")

        self.assertNotIn("old_mode", wave_body)
        self.assertNotIn("LEFT_RIGHT_LINE = old_mode", wave_body)
        self.assertIn("Line_SetTrackModeBumpless(CENTER_LINE_MODE)", wave_body)

        for token in (
            "scaner_set.EdgeIgnore = old_ignore",
            "line_pid_param = old_line",
            "gyroG_pid_param = old_gyro",
            "WavePlateLeft_Flag = 0",
            "WavePlateRight_Flag = 0",
            "barrier_done(0, 0)",
        ):
            self.assertIn(token, wave_body)

    def test_bumpless_track_switch_syncs_pid_history(self):
        self.assertIn("void Line_SetTrackModeBumpless(uint8_t mode);", self.scaner_header)
        body = function_body(self.scaner, "Line_SetTrackModeBumpless")

        for token in (
            "taskENTER_CRITICAL()",
            "LEFT_RIGHT_LINE = mode",
            "getline_error()",
            "line_pid_obj.target = scaner_set.CatchsensorNum",
            "line_pid_obj.measure",
            "line_pid_obj.bias",
            "line_pid_obj.last_bias = line_pid_obj.bias",
            "line_pid_obj.integral = 0.0f",
            "line_pid_obj.last_differential = 0.0f",
            "line_pid_obj.output",
            "taskEXIT_CRITICAL()",
        ):
            self.assertIn(token, body)

        assert_in_order(
            self,
            body,
            "LEFT_RIGHT_LINE = mode",
            "getline_error()",
            "line_pid_obj.bias",
            "line_pid_obj.last_bias = line_pid_obj.bias",
            "line_pid_obj.integral = 0.0f",
            "line_pid_obj.last_differential = 0.0f",
        )

    def test_node_advance_is_the_only_route_mode_handoff(self):
        self.assertNotIn("cross_pass_turn()", self.map)
        self.assertNotIn("LEFT_RIGHT_LINE =", self.map)

        turn_update = function_body(self.map, "cross_turn_update")
        self.assertIn("if (route_need_turn(ad, ad2))", turn_update)
        self.assertIn("cross_stop_turn()", turn_update)
        self.assertIn("cross_run_turn()", turn_update)
        self.assertLess(turn_update.index("cross_stop_turn()"), turn_update.index("cross_node_advance()"))
        self.assertLess(turn_update.index("cross_run_turn()"), turn_update.index("cross_node_advance()"))

        advance = function_body(self.map, "cross_node_advance")
        assert_in_order(
            self,
            advance,
            "nodesr.nowNode = nodesr.nextNode",
            "nodesr.nextNode =",
            "Chassis_SetTargetSpeed(nodesr.nowNode.speed)",
            "Line_SetTrackModeBumpless",
            "Chassis_SetMode(is_Line)",
        )
        self.assertIn("force_center = cross_special_n2_b1()", advance)

        special = function_body(self.map, "cross_special_n2_b1")
        self.assertIn("static uint8_t cross_special_n2_b1(void)", self.map)
        self.assertIn("return 0", special)
        self.assertIn("return 1", special)
        self.assertNotIn("Chassis_SetMode(is_Line)", special)


if __name__ == "__main__":
    unittest.main()
