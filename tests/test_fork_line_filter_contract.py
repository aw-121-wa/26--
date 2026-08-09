from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCANER_SOURCE = ROOT / "Sensor" / "scaner.c"


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


class ForkLineFilterContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = SCANER_SOURCE.read_text(encoding="utf-8")

    def test_go_line_does_not_fuse_fork_samples_into_pid_measurement(self):
        go_line = function_body(self.source, "Go_Line")

        self.assertNotIn("route_has_fork", go_line)
        self.assertNotIn("line_pid_obj.measure * 0.5f", go_line)
        self.assertNotIn("line_pid_obj.target * 0.5f", go_line)

    def test_line_scan_keeps_target_side_hard_selection(self):
        line_scan = function_body(self.source, "Line_Scan")

        for token in (
            "scan_left_line(scaner, sensorNum, edge_ignore, &error, &lednum_tmp)",
            "scan_right_line(scaner, sensorNum, edge_ignore, &error, &lednum_tmp)",
            "scan_center_line(scaner, sensorNum, edge_ignore, &error, &lednum_tmp)",
            "(nodesr.nowNode.flag & LEFT_LINE) == LEFT_LINE",
            "(nodesr.nowNode.flag & RIGHT_LINE) == RIGHT_LINE",
            "(nodesr.nowNode.flag & LiuShui) == LiuShui",
        ):
            self.assertIn(token, line_scan)

    def test_line_scan_rejects_overwide_selected_line_segments(self):
        line_scan = function_body(self.source, "Line_Scan")

        self.assertGreaterEqual(line_scan.count("lednum_tmp > 5"), 2)
        self.assertGreaterEqual(line_scan.count("Scaner.error = 0"), 3)
        self.assertIn("scaner->lineNum > 1", line_scan)
        self.assertIn("scaner->ledNum >= 4", line_scan)


if __name__ == "__main__":
    unittest.main()
