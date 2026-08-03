from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCANER_SOURCE = ROOT / "Sensor" / "scaner.c"
SCANER_HEADER = ROOT / "Sensor" / "scaner.h"
MAP_MESSAGE = ROOT / "App" / "map" / "map_message.c"
SIDE_SOURCE = ROOT / "Sensor" / "line_side_selector.c"
SIDE_HEADER = ROOT / "Sensor" / "line_side_selector.h"


class ScannerRollbackContractTest(unittest.TestCase):
    def test_half_array_selector_is_removed(self):
        self.assertFalse(SIDE_SOURCE.exists())
        self.assertFalse(SIDE_HEADER.exists())

    def test_scanner_uses_legacy_directional_scans_and_full_statistics(self):
        source = SCANER_SOURCE.read_text(encoding="utf-8")
        header = SCANER_HEADER.read_text(encoding="utf-8")

        self.assertNotIn("LineSide_Scan", source)
        self.assertNotIn("line_side_selector.h", header)
        self.assertNotIn("trackLedNum", source + header)
        self.assertNotIn("trackLineNum", source + header)
        self.assertIn("Scaner.lineNum, Scaner.ledNum", source)
        self.assertIn("for (uint8_t i = edge_ignore; i < sensorNum - edge_ignore; i++)", source)
        self.assertIn("for (int8_t i = sensorNum - 1 - edge_ignore; i >= edge_ignore; i--)", source)

    def test_p2_to_n2_keeps_original_right_line_route(self):
        map_message = MAP_MESSAGE.read_text(encoding="utf-8")

        self.assertIn("/*P2 -> N2*/  {N2, RIGHT_LINE", map_message)


if __name__ == "__main__":
    unittest.main()
