from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
MAP_SOURCE = ROOT / "App" / "map" / "map.c"
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


class P4N6ForkGuardContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = MAP_SOURCE.read_text(encoding="utf-8")
        cls.scaner_source = SCANER_SOURCE.read_text(encoding="utf-8")

    def test_shared_parameters_and_route_states_cover_all_three_guards(self):
        for macro, value in (
            ("P4_N6_FORK_PRE_CM", "20.0f"),
            ("P4_N6_FORK_POST_CM", "15.0f"),
            ("N5_N6_FORK_PRE_CM", "20.0f"),
            ("N5_N6_FORK_POST_CM", "15.0f"),
            ("N4_N3_FORK_PRE_CM", "20.0f"),
            ("N4_N3_FORK_POST_CM", "20.0f"),
            ("FORK_GUARD_EDGE_IGNORE", "6"),
        ):
            self.assertRegex(
                self.source,
                rf"#define\s+{macro}\s+{re.escape(value)}",
            )

        for route in (
            "FORK_GUARD_NONE",
            "FORK_GUARD_P4_N6_N5",
            "FORK_GUARD_N5_N6_P4",
            "FORK_GUARD_N4_N3_P3",
        ):
            self.assertIn(route, self.source)

        self.assertIn("ForkGuardRoute_t fork_guard_route", self.source)
        self.assertIn("int8_t fork_guard_saved_edge_ignore", self.source)

    def test_enable_saves_original_value_before_selecting_middle_four_lanes(self):
        enable = function_body(self.source, "fork_guard_enable")

        self.assertIn("if (fork_guard_route != FORK_GUARD_NONE)", enable)
        save = enable.index("fork_guard_saved_edge_ignore")
        select = enable.index("scaner_set.EdgeIgnore = FORK_GUARD_EDGE_IGNORE")
        route = enable.index("fork_guard_route = route")
        bumpless = enable.index("Line_SetTrackModeBumpless(LEFT_RIGHT_LINE)")
        self.assertLess(save, select)
        self.assertLess(select, route)
        self.assertLess(route, bumpless)

    def test_disable_restores_original_value_and_clears_shared_state(self):
        disable = function_body(self.source, "fork_guard_disable")

        self.assertIn("if (fork_guard_route == FORK_GUARD_NONE)", disable)
        restore = disable.index("scaner_set.EdgeIgnore = fork_guard_saved_edge_ignore")
        clear = disable.index("fork_guard_route = FORK_GUARD_NONE")
        bumpless = disable.index("Line_SetTrackModeBumpless(LEFT_RIGHT_LINE)")
        self.assertLess(restore, clear)
        self.assertLess(clear, bumpless)

    def test_update_contains_enable_and_post_release_for_all_three_routes(self):
        update = function_body(self.source, "cross_fork_guard_update")

        for condition in (
            "nodesr.lastNode.nodenum == P4 &&\n"
            "        nodesr.nowNode.nodenum  == N6 &&\n"
            "        nodesr.nextNode.nodenum == N5",
            "nodesr.lastNode.nodenum == N5 &&\n"
            "        nodesr.nowNode.nodenum  == N6 &&\n"
            "        nodesr.nextNode.nodenum == P4",
            "nodesr.lastNode.nodenum == N4 &&\n"
            "        nodesr.nowNode.nodenum  == N3 &&\n"
            "        nodesr.nextNode.nodenum == P3",
        ):
            self.assertIn(condition, update)

        for token in (
            "nodesr.nowNode.step - P4_N6_FORK_PRE_CM",
            "mileage >= P4_N6_FORK_POST_CM",
            "nodesr.nowNode.step - N5_N6_FORK_PRE_CM",
            "mileage >= N5_N6_FORK_POST_CM",
            "nodesr.nowNode.step - N4_N3_FORK_PRE_CM",
            "mileage >= N4_N3_FORK_POST_CM",
            "fork_guard_enable(FORK_GUARD_P4_N6_N5)",
            "fork_guard_enable(FORK_GUARD_N5_N6_P4)",
            "fork_guard_enable(FORK_GUARD_N4_N3_P3)",
            "fork_guard_disable()",
        ):
            self.assertIn(token, update)

        self.assertNotIn("Want2Go", update)
        self.assertNotIn("while", update)
        self.assertNotIn("Chassis_ClearMileage", update)

    def test_reset_and_line_update_use_the_shared_guard_in_order(self):
        reset = function_body(self.source, "Cross_reset")
        self.assertIn("fork_guard_disable()", reset)

        line_update = function_body(self.source, "cross_line_update")
        guard = line_update.index("cross_fork_guard_update()")
        self.assertLess(guard, line_update.index("cross_detect_start()"))
        self.assertLess(guard, line_update.index("cross_arrive_check()"))

    def test_cross_getline_is_full_width_and_arrival_keeps_one_processing_path(self):
        cross_getline = function_body(self.scaner_source, "Cross_getline")
        self.assertNotIn("EdgeIgnore", cross_getline)

        arrive = function_body(self.source, "cross_arrive_check")
        self.assertIn("volatile SCANER *arrival_scaner", arrive)
        self.assertIn("arrival_scaner = &Scaner", arrive)
        self.assertIn("if (fork_guard_route != FORK_GUARD_NONE)", arrive)
        self.assertIn("Cross_getline()", arrive)
        self.assertIn("arrival_scaner = &Cross_Scaner", arrive)
        self.assertIn(
            "arrival_detector_update(arrival_scaner, nodesr.nowNode.flag)",
            arrive,
        )
        self.assertEqual(arrive.count("arrival_detector_update("), 1)


if __name__ == "__main__":
    unittest.main()
