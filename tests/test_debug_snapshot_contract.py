from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class DebugSnapshotContractTest(unittest.TestCase):
    def test_snapshot_contains_required_runtime_fields(self):
        header = (ROOT / "Task" / "debug_snapshot.h").read_text(encoding="utf-8")
        for token in (
            "match_state", "round", "route_index", "last_node", "current_node",
            "next_node", "cross_state", "barrier_type", "chassis_mode",
            "target_speed", "actual_speed", "mileage", "yaw", "pitch", "roll",
            "line_detail", "line_count", "led_count", "vision_status",
            "vision_sequence", "stop_reason", "DebugSnapshot_Update",
        ):
            self.assertIn(token, header)


if __name__ == "__main__":
    unittest.main()
