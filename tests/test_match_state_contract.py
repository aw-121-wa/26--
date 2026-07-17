from pathlib import Path
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


if __name__ == "__main__":
    unittest.main()
