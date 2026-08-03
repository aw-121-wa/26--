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


class CrossTrackingContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = MAP_SOURCE.read_text(encoding="utf-8")
        cls.map_message = MAP_MESSAGE_SOURCE.read_text(encoding="utf-8")

    def test_cross_keeps_line_lost_protection_without_anti_snake(self):
        protect_on = function_body(self.source, "cross_line_protect_on")
        protect_off = function_body(self.source, "cross_line_protect_off")

        self.assertIn("Chassis_EnableLineLostProtection()", protect_on)
        self.assertIn("Chassis_DisableLineLostProtection()", protect_off)
        self.assertNotIn("AntiSnake", protect_on + protect_off)

    def test_simple_arrival_requires_three_consecutive_samples(self):
        self.assertRegex(
            self.source,
            r"#define\s+ARRIVE_CONFIRM_SAMPLES\s+3u",
        )
        update = function_body(self.source, "arrival_detector_update")
        self.assertIn("simple_count", update)
        self.assertIn("ARRIVE_CONFIRM_SAMPLES", update)
        self.assertRegex(update, r"else\s*\n\s*arrival_detector\.simple_count\s*=\s*0")

    def test_multiline_arrival_uses_ordered_phases(self):
        for phase in (
            "ARRIVE_MULTI_WAIT_MULTI",
            "ARRIVE_MULTI_WAIT_SINGLE",
            "ARRIVE_MULTI_WAIT_MULTI_AGAIN",
        ):
            self.assertIn(phase, self.source)

        update = function_body(self.source, "arrival_multiline_update")
        self.assertIn("MUL2SING", update)
        self.assertIn("MUL2MUL", update)
        self.assertIn("s->lineNum == 1", update)
        self.assertGreaterEqual(update.count("s->lineNum > 1"), 2)

    def test_temp_track_switches_once_at_seventy_percent(self):
        self.assertRegex(
            self.source,
            r"#define\s+ROUTE_DETECT_RATIO\s+0\.7f",
        )

        track_switch = function_body(self.source, "cross_track_switch")
        self.assertIn("ROUTE_DETECT_RATIO", track_switch)
        self.assertIn("apply_temp_track_mode", track_switch)
        self.assertIn("Line_SetJunctionHold(0)", track_switch)

        arrive_check = function_body(self.source, "cross_arrive_check")
        self.assertNotIn("TEMP_TRACK_PRIMARY", arrive_check)
        self.assertNotIn("TEMP_TRACK_CLEARANCE", arrive_check)
        self.assertNotIn("temp_switch_mileage", self.source)
        self.assertNotIn("TEMP_TRACK_CLEAR_CM", self.source)

    def test_p2_keeps_half_distance_track_switch(self):
        track_switch = function_body(self.source, "cross_track_switch")
        self.assertIn("route_is_p2_to_n2()", track_switch)
        self.assertIn("ROUTE_HALF_RATIO", track_switch)
        self.assertIn("LEFT_RIGHT_LINE = RIGHT_LINE_MODE", track_switch)

    def test_n5_to_n4_reaches_n4_on_one_multiline_sequence(self):
        self.assertRegex(
            self.map_message,
            r"\{N4,\s*LEFT_LINE\|Temp_L\|MUL2SING,\s*0,\s*120,",
        )
        arrive_check = function_body(self.source, "cross_arrive_check")
        self.assertIn("route_set_arrived()", arrive_check)
        self.assertNotIn("apply_temp_track_mode", arrive_check)

    def test_detector_state_is_reset_with_each_route_phase(self):
        reset = function_body(self.source, "route_phase_reset")
        init = function_body(self.source, "cross_line_init")
        self.assertIn("arrival_detector_reset()", reset)
        self.assertIn("arrival_detector_reset()", init)
        self.assertIn("Line_SetJunctionHold(0)", reset)
        self.assertIn("Line_SetJunctionHold(1)", init)

    def test_junction_hold_is_disabled_at_non_tracking_boundaries(self):
        for function_name in (
            "cross_barrier_update",
            "cross_turn_update",
            "cross_node_advance",
            "cross_route_end",
        ):
            body = function_body(self.source, function_name)
            self.assertIn(
                "Line_SetJunctionHold(0)",
                body,
                f"{function_name} must disable junction hold",
            )


if __name__ == "__main__":
    unittest.main()
