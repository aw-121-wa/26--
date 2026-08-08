from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]


class StagePlatformContractTest(unittest.TestCase):
    def test_stage_descend_uses_p1_departure_flow_for_all_generic_platforms(self):
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
            "Chassis_SetMode(is_Gyro);",
            "motor_all.Gspeed = UPDOWN_SPEED_LOW;",
            "angle.AngleG = getAngleZ();",
            "wait_for_pitch_below(BEGIN_DOWN",
        ):
            self.assertIn(token, body)

    def test_p2_stage_keeps_its_independent_turn_flow(self):
        source = (ROOT / "App" / "barrier" / "barrier.c").read_text(encoding="utf-8")
        p2_start = source.index("BarrierResult_t Stage_P2(void)")
        bridge_start = source.index("BarrierResult_t Barrier_Bridge(void)")
        p2_body = source[p2_start:bridge_start]

        self.assertIn("Chassis_TurnTo_Timeout(", p2_body)


if __name__ == "__main__":
    unittest.main()
