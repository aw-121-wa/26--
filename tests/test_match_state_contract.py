from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]


class MatchStateContractTest(unittest.TestCase):
    def test_two_round_state_machine_and_fault(self):
        header = (ROOT / "Task" / "main_task.h").read_text(encoding="utf-8")
        source = (ROOT / "Task" / "main_task.c").read_text(encoding="utf-8")
        for state in (
            "MATCH_INIT", "ROUND1_PREPARE", "ROUND1_RUNNING", "ROUND1_FINISH",
            "ROUND2_ROUTE_BUILD", "ROUND2_PREPARE", "ROUND2_RUNNING",
            "RETURN_HOME", "MATCH_FINISH", "MATCH_FAULT",
        ):
            self.assertIn(state, header)
            self.assertIn(f"case {state}:", source)
        self.assertIn("Match_SetMissionData", header)
        self.assertIn("RouteCatalog_SelectReturn", source)
        self.assertIn("RouteBuilder_Commit", source)
        self.assertIn("Chassis_GetStopReason", source)

    def test_round2_resets_departure_pose_before_gate_release(self):
        source = (ROOT / "App" / "map" / "map.c").read_text(encoding="utf-8")
        match = re.search(
            r"void mapInit1\(void\)\s*\{(?P<body>.*?)^\}",
            source,
            re.MULTILINE | re.DOTALL,
        )
        self.assertIsNotNone(match)
        body = match.group("body")

        for token in (
            "nodesr.nowNode.nodenum = P2;",
            "nodesr.nowNode.angle = 0.0f;",
            "nodesr.nowNode.function = NONE;",
            "nodesr.nowNode.speed = SPEED0;",
            "nodesr.nowNode.step = 0u;",
            "nodesr.nowNode.flag = CLEFT | RIGHT_LINE;",
        ):
            self.assertIn(token, body)


if __name__ == "__main__":
    unittest.main()
