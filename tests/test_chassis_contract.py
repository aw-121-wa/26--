from pathlib import Path
import re
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

    def test_periodic_protection_only_keeps_roll_and_yaw(self):
        source_dirs = (
            ROOT / "App" / "chassis",
            ROOT / "App" / "map",
            ROOT / "App" / "barrier",
            ROOT / "Task",
        )
        combined = "\n".join(
            path.read_text(encoding="utf-8")
            for source_dir in source_dirs
            for path in source_dir.glob("*.c")
        )
        combined += "\n".join(
            path.read_text(encoding="utf-8")
            for source_dir in source_dirs
            for path in source_dir.glob("*.h")
        )

        for removed in (
            "Chassis_EnableAntiSnake",
            "Chassis_DisableAntiSnake",
            "anti_snake",
            "Chassis_EnableLineLostProtection",
            "Chassis_DisableLineLostProtection",
            "line_lost_guard_update",
            "Chassis_EnableStallProtection",
            "Chassis_DisableStallProtection",
            "stall_guard_update",
            "游龙",
        ):
            self.assertNotIn(removed, combined)

        chassis_c = (ROOT / "App" / "chassis" / "chassis_api.c").read_text(encoding="utf-8")
        chassis_h = (ROOT / "App" / "chassis" / "chassis_api.h").read_text(encoding="utf-8")
        for token in (
            "Chassis_EnableRollProtection",
            "Chassis_EnableYawJumpProtection",
        ):
            self.assertIn(token, chassis_c + chassis_h)
        for token in (
            "yaw_guard_update()",
            "roll_guard_update()",
        ):
            self.assertIn(token, chassis_c)

    def test_line_pid_speed_kp_is_strengthened(self):
        source = (ROOT / "App" / "chassis" / "chassis_api.c").read_text(encoding="utf-8")
        match = re.search(
            r"static void line_pid_by_speed\(float speed\)\n\{(?P<body>.*?)^\}",
            source,
            re.MULTILINE | re.DOTALL,
        )
        self.assertIsNotNone(match)
        body = match.group("body")

        for token in (
            "line_pid_param.kp = 6.0f;",
            "line_pid_param.kp = 10.0f;",
            "line_pid_param.kp = 12.0f;",
            "line_pid_param.kp = 13.0f;",
            "line_pid_param.kp = 20.0f;",
        ):
            self.assertIn(token, body)


if __name__ == "__main__":
    unittest.main()
