from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]


def function_body(source: str, name: str) -> str:
    match = re.search(
        rf"\b(?:static\s+)?(?:void|uint8_t|float)\s+{re.escape(name)}\s*\([^)]*\)\s*\{{",
        source,
    )
    if match is None:
        raise AssertionError(f"missing function {name}")

    depth = 0
    for index in range(match.end() - 1, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[match.end():index]
    raise AssertionError(f"unterminated function {name}")


class P8FinishContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.route = (ROOT / "App" / "map" / "route_catalog.c").read_text(encoding="utf-8")
        cls.barrier = (ROOT / "App" / "barrier" / "barrier.c").read_text(encoding="utf-8")

    def test_first_round_routes_stop_after_p8_mainline(self):
        for name, macro in (
            ("round1_highscore", "ROUTE_TO_HIGH_SCORE"),
            ("round1_highscore_d2", "ROUTE_HIGH_FROM_N13"),
        ):
            match = re.search(
                rf"static const uint8_t {name}\[\] = \{{(.*?)\n\}};",
                self.route,
                re.DOTALL,
            )
            self.assertIsNotNone(match)
            body = match.group(1)
            self.assertEqual(body.count("ROUTE_END"), 1)
            self.assertIn(f"{macro},\n    ROUTE_END", body)
            self.assertNotIn("ROUTE_P8_RETURN_UPPER", body)
            self.assertNotIn("D5, N3, N4, B3, N2, P2", body)

    def test_high_mountain_finishes_after_voice_wait_and_locks_stop(self):
        self.assertIn("#define HIGH_MOUNTAIN_END_VOICE_WAIT_MS 3000u", self.barrier)
        body = function_body(self.barrier, "Barrier_HighMountain")
        stages = (
            "barrier_platform_start_gesture()",
            "barrier_reverse_distance(6.0f, 12.0f, heading)",
            "barrier_play_platform_voice()",
            "vTaskDelay(\n        pdMS_TO_TICKS(\n            HIGH_MOUNTAIN_END_VOICE_WAIT_MS))",
            "high_mountain_finish_stop()",
        )
        positions = [body.index(stage) for stage in stages]
        self.assertEqual(positions, sorted(positions))
        for forbidden in (
            "Chassis_Turn_180_Blocking()",
            "mpuZreset(",
            "barrier_platform_center()",
            "high_mountain_descend(",
            "Chassis_EnableLineLostProtection()",
            "barrier_complete(",
        ):
            self.assertNotIn(forbidden, body)

    def test_finish_helper_disables_chassis_and_brakes_forever(self):
        body = function_body(self.barrier, "high_mountain_finish_stop")
        self.assertIn("Chassis_SetMode(is_No)", body)
        self.assertIn("CarBrake()", body)
        self.assertIn("while (1)", body)
        self.assertIn("vTaskDelay(pdMS_TO_TICKS(100u))", body)


if __name__ == "__main__":
    unittest.main()
