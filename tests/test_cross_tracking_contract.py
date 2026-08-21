from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
MAP_SOURCE = ROOT / "App" / "map" / "map.c"
MAP_MESSAGE_SOURCE = ROOT / "App" / "map" / "map_message.c"


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
        cls.map_message = MAP_MESSAGE_SOURCE.read_text(encoding="utf-8")

    def test_cross_keeps_line_lost_protection_without_anti_snake(self):
        protect_on = function_body(self.source, "cross_line_protect_on")
        protect_off = function_body(self.source, "cross_line_protect_off")

        self.assertIn("Chassis_EnableLineLostProtection()", protect_on)
        self.assertIn("Chassis_DisableLineLostProtection()", protect_off)
        self.assertNotIn("AntiSnake", protect_on + protect_off)

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

    def test_node_reentry_protection_precedes_all_arrival_sources(self):
        self.assertRegex(self.source, r"#define\s+NODE_REENTRY_CM\s+8\.0f")

        arrive_check = function_body(self.source, "cross_arrive_check")
        guard = re.search(
            r"/\* 节点重入保护.*?\*/\s*\n\s*"
            r"if \(fabsf\(Chassis_GetMileage\(\) - node_entry_mileage\) "
            r"< NODE_REENTRY_CM\)\s*\n\s*return;",
            arrive_check,
            re.DOTALL,
        )
        self.assertIsNotNone(guard)

        guard_index = guard.start()
        self.assertLess(guard_index, arrive_check.index("getline_error()"))
        self.assertLess(guard_index, arrive_check.index("arrival_detector_update"))
        self.assertLess(guard_index, arrive_check.index("nodesr.nowNode.step * 1.2f"))

    def test_node_advance_resets_arrival_detector(self):
        advance = function_body(self.source, "cross_node_advance")
        self.assertIn("arrival_detector_reset()", advance)

    def test_arrival_detection_waits_until_seventy_percent_of_each_route(self):
        """Prevent an N8 junction pattern from advancing the N8->N12 route early."""
        self.assertRegex(
            self.source,
            r"#define\s+ROUTE_DETECT_RATIO\s+0\.7f",
        )

        detect_start = function_body(self.source, "cross_detect_start")
        self.assertIn("ROUTE_DETECT_RATIO * nodesr.nowNode.step", detect_start)

        line_update = function_body(self.source, "cross_line_update")
        self.assertLess(
            line_update.index("cross_detect_start()"),
            line_update.index("cross_arrive_check()"),
        )

    def test_plain_multiline_route_cannot_bypass_sensor_confirmation_with_pitch(self):
        """A flat N8->N12 leg must reach N12 through MUL2MUL, not IMU pitch."""
        arrive_check = function_body(self.source, "cross_arrive_check")
        self.assertNotIn("imu.pitch", arrive_check)

    def test_p3_n3_d4_stop_turn_uses_local_forward_compensation(self):
        self.assertRegex(
            self.source,
            r"#define\s+P3_N3_TURN_FORWARD_CM\s+25\.0f",
        )
        self.assertRegex(
            self.source,
            r"#define\s+P3_N3_POST_TURN_FORWARD_CM\s+8\.0f",
        )

        stop_turn = function_body(self.source, "cross_stop_turn")
        special = re.search(
            r"if\s*\(\s*nodesr\.lastNode\.nodenum\s*==\s*P3\s*&&"
            r"\s*nodesr\.nowNode\.nodenum\s*==\s*N3\s*&&"
            r"\s*nodesr\.nextNode\.nodenum\s*==\s*D4\s*\)\s*\{"
            r"(.*?)\n\s*\}\s*",
            stop_turn,
            re.DOTALL,
        )
        self.assertIsNotNone(special)
        special_body = re.sub(r"\s+", "", special.group(1))

        expected_sequence = [
            "Chassis_DriveDistance_Blocking(is_Gyro,P3_N3_TURN_FORWARD_CM,SPEED1,nodesr.nowNode.angle)",
            "CarBrake()",
            "vTaskDelay(DELAY_SHORT)",
            "turn_amt=need2turn(nodesr.nowNode.angle,nodesr.nextNode.angle)",
            "Chassis_Turn_By_StopGyro_Blocking(compensated,getAngleZ())",
            "CarBrake()",
            "vTaskDelay(DELAY_SHORT)",
            "Chassis_DriveDistance_Blocking(is_Gyro,P3_N3_POST_TURN_FORWARD_CM,SPEED1,nodesr.nextNode.angle)",
            "return;",
        ]
        positions = []
        cursor = 0
        for token in expected_sequence:
            position = special_body.index(token, cursor)
            positions.append(position)
            cursor = position + len(token)
        self.assertEqual(positions, sorted(positions))

        self.assertLess(
            special.start(),
            stop_turn.index("if (nodesr.nowNode.nodenum == N18)"),
        )
        self.assertIn(
            "Chassis_DriveDistance_Blocking(is_Gyro, drive_cm, SPEED1, lock_angle)",
            stop_turn,
        )

    def test_n8_route_keeps_140_degree_heading_until_n12(self):
        route = re.search(r"u8 route\[100\]\s*=\s*\{(.*?)\};", self.source, re.DOTALL)
        self.assertIsNotNone(route)
        self.assertIn("N3, N8, N12, N16", route.group(1))

        n3 = re.search(r"/\* N3 .*?\*/(.*?)/\* N4", self.map_message, re.DOTALL)
        n8 = re.search(r"/\* N8 .*?\*/(.*?)/\* C1", self.map_message, re.DOTALL)
        n12 = re.search(r"/\* N12 .*?\*/(.*?)/\* N13", self.map_message, re.DOTALL)
        self.assertIsNotNone(n3)
        self.assertIsNotNone(n8)
        self.assertIsNotNone(n12)
        self.assertRegex(n3.group(1), r"\{N8,.*?,\s*140,")
        self.assertRegex(n8.group(1), r"\{N12,.*?,\s*140,\s*150,")
        self.assertRegex(n12.group(1), r"\{N16,.*?,\s*90,")


if __name__ == "__main__":
    unittest.main()
