from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]


class StagePlatformContractTest(unittest.TestCase):
    def test_stage_descend_keeps_line_departure_flow_for_all_generic_platforms(self):
        source = (ROOT / "App" / "barrier" / "barrier.c").read_text(encoding="utf-8")
        match = re.search(
            r"case STAGE_DESCEND:\s*\{(?P<body>.*?)\n\s*\}\s*\n\s*default:",
            source,
            re.DOTALL,
        )
        self.assertIsNotNone(match)
        body = match.group("body")

        self.assertNotIn("nodesr.nowNode.nodenum == P1", body)
        for token in (
            "Chassis_MotorControl(is_Line, UPDOWN_SPEED_LOW, UPDOWN_SPEED_LOW, 0.0f);",
            "Chassis_SetTargetSpeed(UPDOWN_SPEED_LOW);",
            "while (imu.pitch > BEGIN_DOWN)",
            "line_mode_reset_by_flag(nodesr.nextNode.flag);",
        ):
            self.assertIn(token, body)

    def test_p2_stage_keeps_its_independent_turn_flow(self):
        source = (ROOT / "App" / "barrier" / "barrier.c").read_text(encoding="utf-8")
        p2_start = source.index("void Stage_P2(void)")
        bridge_start = source.index("void Barrier_Bridge(void)")
        p2_body = source[p2_start:bridge_start]

        self.assertIn("Chassis_Turn_180_Blocking(", p2_body)
        self.assertIn("barrier_platform_center()", p2_body)


if __name__ == "__main__":
    unittest.main()
