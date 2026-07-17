from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class CrossBarrierContractTest(unittest.TestCase):
    def test_cross_has_overrun_and_all_special_flags(self):
        text = (ROOT / "App" / "map" / "map.c").read_text(encoding="utf-8")
        for token in (
            "ROUTE_SEARCH_RATIO", "ROUTE_FAULT_RATIO", "RESTMPUZ", "SLOWDOWN",
            "DRIFT", "L_follow", "R_follow", "INGNORE", "MAP_NODE_INDEX_INVALID",
            "CHASSIS_STOP_ROUTE_INVALID", "MAP_TURN_TIMEOUT_MS",
        ):
            self.assertIn(token, text)

    def test_all_barriers_are_dispatched(self):
        text = (ROOT / "App" / "map" / "map.c").read_text(encoding="utf-8")
        for barrier in (
            "LBHill", "SM", "View", "View1", "BACK", "BSoutPole", "QQB",
            "DOOR", "BHM", "IGNORE", "UNDER", "Special_node", "DOOR1",
        ):
            self.assertIn(f"case {barrier}:", text)

        header = (ROOT / "App" / "barrier" / "barrier.h").read_text(encoding="utf-8")
        for api in (
            "Barrier_DoubleHill", "Barrier_SwordMountain", "Barrier_View",
            "Barrier_Back", "Barrier_SouthPole", "Barrier_Seesaw",
            "Barrier_HighMountain", "Barrier_Under", "Barrier_SpecialNode",
            "Barrier_Door",
        ):
            self.assertIn(api, header)


if __name__ == "__main__":
    unittest.main()
