from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
VISION_SOURCE = ROOT / "App" / "vision" / "vision_api.c"
VISION_HEADER = ROOT / "App" / "vision" / "vision_api.h"


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


class TrafficLightServoContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = VISION_SOURCE.read_text(encoding="utf-8")
        cls.header = VISION_HEADER.read_text(encoding="utf-8")

    def test_traffic_light_scan_does_not_move_camera_servo(self):
        scan = function_body(self.source, "Vision_ScanSingleSide")

        self.assertNotIn("Lsc16_RunActionGroupBlocking", scan)
        self.assertNotIn("LSC16_ACTION_CAMERA_LEFT", scan)
        self.assertNotIn("LSC16_ACTION_CAMERA_RIGHT", scan)
        self.assertNotIn("VISION_SERVO_SETTLE_MS", scan)
        self.assertNotIn("vTaskDelay", scan)
        self.assertNotIn("VISION_SERVO_SETTLE_MS", self.header)

    def test_non_traffic_vision_request_keeps_camera_servo_path(self):
        request = function_body(self.source, "Vision_Request")

        self.assertIn("mode != VISION_MODE_TRAFFIC_LIGHT", request)
        servo_guard = re.search(
            r"if\s*\(\s*mode\s*!=\s*VISION_MODE_TRAFFIC_LIGHT\s*\)\s*\{(.*?)\n\s*\}",
            request,
            re.DOTALL,
        )
        self.assertIsNotNone(servo_guard)
        self.assertIn("Lsc16_RunActionGroupBlocking", servo_guard.group(1))


if __name__ == "__main__":
    unittest.main()
