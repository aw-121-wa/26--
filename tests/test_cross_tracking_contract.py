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


class CrossTrackingContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = MAP_SOURCE.read_text(encoding="utf-8")

    def test_cross_does_not_add_secondary_line_protection(self):
        self.assertNotIn("cross_line_protect_on", self.source)
        self.assertNotIn("cross_line_protect_off", self.source)
        self.assertNotIn("AntiSnake", self.source)
        self.assertNotIn("LineLostProtection", self.source)

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

    def test_temp_track_first_arrival_enters_clearance_phase(self):
        self.assertRegex(
            self.source,
            r"#define\s+TEMP_TRACK_CLEAR_CM\s+10\.0f",
        )
        arrive_check = function_body(self.source, "cross_arrive_check")
        self.assertIn("TEMP_TRACK_PRIMARY", arrive_check)
        self.assertIn("TEMP_TRACK_CLEARANCE", arrive_check)
        self.assertIn("apply_temp_track_mode", arrive_check)
        self.assertIn("temp_switch_mileage", arrive_check)

        clearance = function_body(self.source, "temp_track_clearance_done")
        self.assertIn("TEMP_TRACK_CLEAR_CM", clearance)
        self.assertIn("TEMP_TRACK_FINAL", clearance)

        fixed_switch = function_body(self.source, "cross_track_switch")
        self.assertNotIn("Temp_L", fixed_switch)
        self.assertNotIn("Temp_R", fixed_switch)
        self.assertNotIn("Temp_LiuShui", fixed_switch)

    def test_detector_state_is_reset_with_each_route_phase(self):
        reset = function_body(self.source, "route_phase_reset")
        init = function_body(self.source, "cross_line_init")
        self.assertIn("arrival_detector_reset()", reset)
        self.assertIn("arrival_detector_reset()", init)
        self.assertIn("temp_track_reset", init)


if __name__ == "__main__":
    unittest.main()
