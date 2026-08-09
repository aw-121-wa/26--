from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
MAP_MESSAGE = ROOT / "App" / "map" / "map_message.c"
MAP_SOURCE = ROOT / "App" / "map" / "map.c"
BARRIER_HEADER = ROOT / "App" / "barrier" / "barrier.h"
BARRIER_SOURCE = ROOT / "App" / "barrier" / "barrier.c"


def function_body(source: str, name: str) -> str:
    start = source.index(f"void {name}(void)")
    opening = source.index("{", start)
    depth = 0
    for index in range(opening, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[start:index + 1]
    raise AssertionError(f"unterminated function {name}")


class DoorWaitContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = MAP_MESSAGE.read_text(encoding="utf-8")
        cls.map_source = MAP_SOURCE.read_text(encoding="utf-8")
        cls.barrier_header = BARRIER_HEADER.read_text(encoding="utf-8")
        cls.barrier_source = BARRIER_SOURCE.read_text(encoding="utf-8")

    def test_all_d1_to_d5_edges_wait_at_the_door(self):
        expected = (
            ("N10", "DLEFT|RIGHT_LINE", "90", "90", "SPEED3"),
            ("N8", "DRIGHT|DLEFT", "140", "75", "SPEED0"),
            ("N8", "CLEFT|DLEFT", "35", "80", "SPEED0"),
            ("N12", "AWHITE|RESTMPUZ", "90", "90", "SPEED1"),
            ("N3", "CLEFT|LEFT_LINE|MUL2MUL", "-45", "60", "SPEED0"),
            ("N5", "STOPTURN|CLEFT", "-140", "150", "SPEED0"),
            ("N3", "DRIGHT|DLEFT", "-90", "95", "SPEED0"),
            ("N13", "DRIGHT|DLEFT|CLEFT|CRIGHT|DRIFT", "120", "20", "SPEED1"),
        )

        for node, flag, angle, step, speed in expected:
            initializer = rf"\{{\s*{node}\s*,\s*{re.escape(flag)}\s*,\s*{angle}\s*,\s*{step}\s*,\s*{speed}\s*,\s*DOOR\s*\}}"
            self.assertRegex(self.source, initializer)

        self.assertEqual(len(re.findall(r"\{[^\n{}]*,\s*DOOR\s*\}", self.source)), 8)

    def test_door_handler_reaches_stop_line_before_waiting_without_vision(self):
        self.assertIn("void Barrier_Door(void);", self.barrier_header)
        self.assertRegex(self.barrier_source, r"#define\s+DOOR_WAIT_MS\s+3000u")
        self.assertRegex(self.barrier_source, r"#define\s+DOOR_STOP_LED_NUM\s+8u")
        self.assertRegex(self.barrier_source, r"#define\s+DOOR_APPROACH_TIMEOUT_MS\s+5000u")
        self.assertRegex(
            self.barrier_source,
            re.compile(
                r"static uint8_t barrier_wait_door_stop_line\(void\).*?"
                r"Scaner\.ledNum\s*<\s*DOOR_STOP_LED_NUM.*?"
                r"getline_error\(\).*?"
                r"Chassis_IsStopLocked\(\)",
                re.DOTALL,
            ),
        )

        body = function_body(self.barrier_source, "Barrier_Door")
        self.assertLess(body.index("Chassis_SetMode(is_Line);"), body.index("barrier_wait_door_stop_line()"))
        self.assertLess(body.index("barrier_wait_door_stop_line()"), body.index("CarBrake();"))
        self.assertLess(body.index("CarBrake();"), body.index("vTaskDelay(pdMS_TO_TICKS(DOOR_WAIT_MS));"))
        self.assertIn("nodesr.nowNode.function = NONE;", body)
        self.assertIn("nodesr.flag |= NODE_ARRIVED_FLAG;", body)
        self.assertIn("nodesr.flag &= (uint8_t)(~NODE_ARRIVED_FLAG);", body)
        self.assertLess(
            body.index("nodesr.flag &= (uint8_t)(~NODE_ARRIVED_FLAG);"),
            body.index("barrier_wait_door_stop_line()"),
        )
        self.assertEqual(body.count("barrier_door_fail();"), 2)

        failure = function_body(self.barrier_source, "barrier_door_fail")
        self.assertIn("nodesr.flag &= (uint8_t)(~NODE_ARRIVED_FLAG);", failure)
        self.assertIn("Chassis_ForceStop(CHASSIS_STOP_BARRIER_FAILED);", failure)
        for forbidden in ("LineSensor_", "door_route", "Vision", "vision", "update_rout"):
            self.assertNotIn(forbidden, body)

    def test_door_enums_dispatch_to_the_wait_handler(self):
        self.assertRegex(
            self.map_source,
            r"case DOOR:\s*\n\s*case DOOR1:\s*\n\s*Barrier_Door\(\);",
        )

    def test_p8_has_only_c10_and_c10_is_long_wave_plate(self):
        p8_block = re.search(
            r"/\* P8 .*?\*/(.*?)(?=/\* N11 )", self.source, re.DOTALL
        )
        self.assertIsNotNone(p8_block)
        p8_edges = re.findall(r"\{\s*([A-Z]\w*)\s*,", p8_block.group(1))
        self.assertEqual(p8_edges, ["C10"])

        n22_block = re.search(
            r"/\* N22 .*?\*/(.*?)(?=/\* C6 )", self.source, re.DOTALL
        )
        self.assertIsNotNone(n22_block)
        self.assertRegex(n22_block.group(1), r"\{\s*C10\s*,[^\n]*,\s*BLBL\s*\}")


if __name__ == "__main__":
    unittest.main()
