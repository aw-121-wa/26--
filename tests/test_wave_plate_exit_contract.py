from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
BARRIER_SOURCE = ROOT / "App" / "barrier" / "barrier.c"
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


class WavePlateExitContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = BARRIER_SOURCE.read_text(encoding="utf-8")
        cls.scaner = SCANER_SOURCE.read_text(encoding="utf-8")
        cls.scaner_header = SCANER_HEADER.read_text(encoding="utf-8")

    def test_wave_exit_continues_line_without_marking_node_arrived(self):
        wave = function_body(self.source, "Barrier_WavedPlate")
        continuation = function_body(self.source, "barrier_continue_after_wave")

        self.assertIn("barrier_continue_after_wave()", wave)
        self.assertNotIn("barrier_done(0, 0)", wave)
        self.assertIn("Chassis_ClearMileage()", continuation)
        self.assertIn("nodesr.nowNode.function = 0", continuation)
        self.assertIn(
            "nodesr.flag &= (uint8_t)(~NODE_ARRIVED_FLAG)",
            continuation,
        )
        self.assertNotIn("nodesr.flag |= NODE_ARRIVED_FLAG", continuation)

    def test_wave_exit_resynchronizes_line_pid_before_continuing(self):
        wave = function_body(self.source, "Barrier_WavedPlate")

        self.assertRegex(wave, r"line_pid_param\.kd\s*=\s*15(?:\.0f)?")
        self.assertGreaterEqual(
            wave.count("Line_SetTrackModeBumpless(CENTER_LINE_MODE)"),
            2,
        )
        self.assertIn("Line_SetTrackModeBumpless(old_mode)", wave)
        self.assertLess(
            wave.index("line_pid_param = old_line"),
            wave.index("Line_SetTrackModeBumpless(old_mode)"),
        )
        self.assertLess(
            wave.index("scaner_set.EdgeIgnore = old_ignore"),
            wave.index("Line_SetTrackModeBumpless(old_mode)"),
        )

        self.assertIn(
            "void Line_SetTrackModeBumpless(uint8_t mode);",
            self.scaner_header,
        )
        handoff = function_body(self.scaner, "Line_SetTrackModeBumpless")
        for token in (
            "LEFT_RIGHT_LINE = mode",
            "getline_error()",
            "line_pid_obj.last_bias = line_pid_obj.bias",
            "line_pid_obj.integral = 0.0f",
            "line_pid_obj.last_differential = 0.0f",
        ):
            self.assertIn(token, handoff)


if __name__ == "__main__":
    unittest.main()
