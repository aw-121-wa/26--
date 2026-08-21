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

    def test_guard_uses_first_version_parameters(self):
        self.assertRegex(
            self.source,
            r"#define\s+P4_N6_FORK_PRE_CM\s+15\.0f",
        )
        self.assertRegex(
            self.source,
            r"#define\s+P4_N6_FORK_POST_CM\s+15\.0f",
        )
        self.assertRegex(
            self.source,
            r"#define\s+P4_N6_FORK_EDGE_IGNORE\s+6",
        )
        self.assertIn("p4_n6_fork_guard_active", self.source)
        self.assertIn("p4_n6_saved_edge_ignore", self.source)

    def test_enable_saves_original_value_before_selecting_middle_four_lanes(self):
        enable = function_body(self.source, "p4_n6_fork_guard_enable")

        self.assertIn("if (p4_n6_fork_guard_active)", enable)
        save = enable.index("p4_n6_saved_edge_ignore")
        select = enable.index("scaner_set.EdgeIgnore = P4_N6_FORK_EDGE_IGNORE")
        self.assertLess(save, select)
        self.assertIn("p4_n6_saved_edge_ignore = scaner_set.EdgeIgnore", enable)
        self.assertIn("p4_n6_fork_guard_active = 1u", enable)
        self.assertIn("Line_SetTrackModeBumpless(LEFT_RIGHT_LINE)", enable)

    def test_disable_restores_original_value_and_is_bumpless(self):
        disable = function_body(self.source, "p4_n6_fork_guard_disable")

        self.assertIn("if (!p4_n6_fork_guard_active)", disable)
        self.assertIn("scaner_set.EdgeIgnore = p4_n6_saved_edge_ignore", disable)
        self.assertIn("p4_n6_fork_guard_active = 0u", disable)
        self.assertIn("Line_SetTrackModeBumpless(LEFT_RIGHT_LINE)", disable)

    def test_update_enables_near_n6_and_keeps_guard_across_node_transition(self):
        update = function_body(self.source, "cross_p4_n6_fork_guard_update")

        self.assertIn("mileage = fabsf(Chassis_GetMileage())", update)
        self.assertIn(
            "nodesr.lastNode.nodenum == P4 &&\n"
            "        nodesr.nowNode.nodenum  == N6 &&\n"
            "        nodesr.nextNode.nodenum == N5",
            update,
        )
        self.assertIn("nodesr.nowNode.step - P4_N6_FORK_PRE_CM", update)
        self.assertIn("mileage >= enable_distance", update)
        self.assertIn("p4_n6_fork_guard_enable()", update)

        post_start = update.index("nodesr.lastNode.nodenum == N6")
        post = update[post_start:]
        self.assertIn("nodesr.nowNode.nodenum  == N5", post)
        self.assertIn("mileage >= P4_N6_FORK_POST_CM", post)
        self.assertIn("p4_n6_fork_guard_disable()", post)

        self.assertNotIn("Want2Go", update)
        self.assertNotIn("while", update)
        self.assertNotIn("Chassis_ClearMileage", update)

    def test_update_is_called_during_normal_line_tracking_before_arrival_detection(self):
        line_update = function_body(self.source, "cross_line_update")

        guard = line_update.index("cross_p4_n6_fork_guard_update()")
        self.assertLess(guard, line_update.index("cross_detect_start()"))
        self.assertLess(guard, line_update.index("cross_arrive_check()"))

    def test_reset_clears_a_stale_guard_without_changing_arrival_sensor_path(self):
        reset = function_body(self.source, "Cross_reset")
        self.assertIn("p4_n6_fork_guard_disable()", reset)

        cross_getline = function_body(self.scaner_source, "Cross_getline")
        self.assertNotIn("EdgeIgnore", cross_getline)

        arrive = function_body(self.source, "cross_arrive_check")
        self.assertNotIn("EdgeIgnore", arrive)


if __name__ == "__main__":
    unittest.main()
