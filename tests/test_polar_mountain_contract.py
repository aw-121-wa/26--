from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]


def function_body(source: str, name: str) -> str:
    match = re.search(
        rf"\b(?:void|uint8_t|float)\s+{re.escape(name)}\s*\([^)]*\)\s*\{{",
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


class PolarMountainContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.header = (ROOT / "App" / "barrier" / "barrier.h").read_text(encoding="utf-8")
        cls.barrier = (ROOT / "App" / "barrier" / "barrier.c").read_text(encoding="utf-8")
        cls.map_source = (ROOT / "App" / "map" / "map.c").read_text(encoding="utf-8")

    def test_public_api_and_dispatch_are_wired(self):
        self.assertIn("void Barrier_SouthPole(void);", self.header)
        self.assertIn("void Barrier_HighMountain(void);", self.header)
        self.assertRegex(
            self.map_source,
            r"case\s+BSoutPole\s*:\s*Barrier_SouthPole\(\)\s*;\s*break\s*;",
        )
        self.assertRegex(
            self.map_source,
            r"case\s+BHM\s*:\s*Barrier_HighMountain\(\)\s*;\s*break\s*;",
        )

    def test_south_pole_keeps_motion_sequence_and_failure_exit(self):
        body = function_body(self.barrier, "Barrier_SouthPole")
        stages = (
            "south_pole_capture_heading",
            "south_pole_ascend",
            "while (Infrared_ahead == 0)",
            "barrier_reverse_distance",
            "Chassis_Turn_180_Blocking",
            "south_pole_descend",
            "barrier_complete",
        )
        positions = [body.index(stage) for stage in stages]
        self.assertEqual(positions, sorted(positions))
        self.assertIn("barrier_fail", body)

    def test_high_mountain_keeps_two_slope_sequence_and_failure_exit(self):
        body = function_body(self.barrier, "Barrier_HighMountain")
        stages = (
            "high_mountain_first_ascend",
            "high_mountain_second_ascend",
            "while (Infrared_ahead == 0)",
            "barrier_reverse_distance",
            "Chassis_Turn_180_Blocking",
            "high_mountain_descend",
            "barrier_complete",
        )
        positions = [body.index(stage) for stage in stages]
        self.assertEqual(positions, sorted(positions))
        self.assertIn("barrier_fail", body)

    def test_south_pole_uses_platform_lsc16_sequence(self):
        body = function_body(self.barrier, "Barrier_SouthPole")
        stages = (
            "while (Infrared_ahead == 0)",
            "Lsc16_RunActionGroupBlocking(LSC16_ACTION_BARRIER_DETECTED",
            "Chassis_Turn_180_Blocking",
            "Lsc16_RunActionGroupBlocking(LSC16_ACTION_TURN_DONE",
            "south_pole_descend",
        )
        positions = [body.index(stage) for stage in stages]
        self.assertEqual(positions, sorted(positions))
        self.assertIn("LSC16_WAIT_PLATFORM_MS", body)
        self.assertIn("LSC16_WAIT_STAND_MS", body)

    def test_high_mountain_uses_platform_lsc16_sequence(self):
        body = function_body(self.barrier, "Barrier_HighMountain")
        stages = (
            "while (Infrared_ahead == 0)",
            "Lsc16_RunActionGroupBlocking(LSC16_ACTION_BARRIER_DETECTED",
            "Chassis_Turn_180_Blocking",
            "Lsc16_RunActionGroupBlocking(LSC16_ACTION_TURN_DONE",
            "high_mountain_descend",
        )
        positions = [body.index(stage) for stage in stages]
        self.assertEqual(positions, sorted(positions))
        self.assertIn("LSC16_WAIT_PLATFORM_MS", body)
        self.assertIn("LSC16_WAIT_STAND_MS", body)

    def test_south_pole_and_high_mountain_use_platform_infrared_without_debug(self):
        for name in ("Barrier_SouthPole", "Barrier_HighMountain"):
            body = function_body(self.barrier, name)
            self.assertIn("while (Infrared_ahead == 0)", body)
            self.assertNotIn("barrier_wait_front_infrared", body)
            for debug_symbol in (
                "HAL_UART_Transmit",
                "snprintf",
                "g_barrier_step",
            ):
                self.assertNotIn(debug_symbol, body)

    def test_failure_path_restores_state_and_locks_cross(self):
        fail_body = function_body(self.barrier, "barrier_fail")
        self.assertIn("barrier_motion_restore", fail_body)
        self.assertIn("CHASSIS_STOP_BARRIER_FAILED", fail_body)
        self.assertRegex(fail_body, r"nodesr\.flag\s*&=")

        cross_body = function_body(self.map_source, "Cross")
        self.assertIn("Chassis_IsStopLocked()", cross_body)

        complete_body = function_body(self.barrier, "barrier_complete")
        self.assertIn("Chassis_IsStopLocked()", complete_body)
        self.assertIn("barrier_fail", complete_body)
        self.assertLess(
            complete_body.index("Chassis_IsStopLocked()"),
            complete_body.index("barrier_done"),
        )
        self.assertLess(complete_body.index("barrier_fail"), complete_body.index("barrier_done"))

    def test_cross_rechecks_stop_lock_before_advancing_node(self):
        body = function_body(self.map_source, "cross_turn_update")
        self.assertIn("Chassis_IsStopLocked()", body)
        self.assertLess(body.index("Chassis_IsStopLocked()"), body.index("route_clear_arrived()"))
        self.assertLess(body.index("route_clear_arrived()"), body.index("cross_node_advance()"))

    def test_high_mountain_first_descent_confirms_slope_entry(self):
        body = function_body(self.barrier, "high_mountain_descend")
        self.assertLess(
            body.index("barrier_wait_pitch_below(BEGIN_DOWN"),
            body.index("barrier_wait_pitch_above(AFTER_DOWN"),
        )

    def test_scanner_waits_take_a_fresh_sample_before_testing_thresholds(self):
        for helper in ("barrier_wait_led_at_least", "barrier_wait_led_below"):
            body = function_body(self.barrier, helper)
            self.assertLess(body.index("getline_error();"), body.index("while"))

        first_ascent = function_body(self.barrier, "high_mountain_first_ascend")
        self.assertLess(first_ascent.index("getline_error();"), first_ascent.index("while"))

    def test_platform_infrared_helper_is_not_used_by_polar_or_mountain(self):
        self.assertNotIn("barrier_wait_front_infrared", self.barrier)
        self.assertNotIn("BARRIER_INFRARED_TIMEOUT_MS", self.barrier)

    def test_obstacle_functions_do_not_depend_on_removed_task_features(self):
        bodies = function_body(self.barrier, "Barrier_SouthPole") + function_body(
            self.barrier, "Barrier_HighMountain"
        )
        for symbol in (
            "Rudder_control",
            "Rudder_Init",
            "Robot_Work",
            "send_play_specified_command",
            "treasure",
            "flag_clue",
            "update_rout_by_treasure",
        ):
            self.assertNotIn(symbol, bodies)


if __name__ == "__main__":
    unittest.main()
