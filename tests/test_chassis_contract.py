from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class ChassisContractTest(unittest.TestCase):
    def test_timeout_and_stop_contract_is_declared(self):
        text = (ROOT / "App" / "chassis" / "chassis_api.h").read_text(encoding="utf-8")
        for token in (
            "CHASSIS_ACTION_OK",
            "CHASSIS_ACTION_TIMEOUT",
            "CHASSIS_ACTION_STOPPED",
            "CHASSIS_ACTION_SENSOR_FAULT",
            "CHASSIS_STOP_STALL",
            "CHASSIS_STOP_MOTION_TIMEOUT",
            "CHASSIS_STOP_ROUTE_INVALID",
            "CHASSIS_STOP_VISION_TIMEOUT",
            "CHASSIS_STOP_BARRIER_FAILED",
            "timeout_ms",
        ):
            self.assertIn(token, text)

    def test_stall_thresholds_are_explicit(self):
        text = (ROOT / "App" / "chassis" / "chassis_api.c").read_text(encoding="utf-8")
        for token in (
            "STALL_TARGET_MIN",
            "STALL_MEASURE_MAX",
            "STALL_OUTPUT_RATIO",
            "STALL_CONFIRM_COUNT",
            "stall_guard_update",
        ):
            self.assertIn(token, text)


if __name__ == "__main__":
    unittest.main()
