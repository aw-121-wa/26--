from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]


def function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    brace = source.index("{", start)
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[brace + 1:index]
    raise AssertionError(f"unterminated function: {signature}")


class StageTurnStabilityContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.api = (ROOT / "App" / "chassis" / "chassis_api.c").read_text(
            encoding="utf-8"
        )
        cls.turn = (ROOT / "Task" / "turn.c").read_text(encoding="utf-8")
        cls.barrier = (ROOT / "App" / "barrier" / "barrier.c").read_text(
            encoding="utf-8"
        )

    def test_platform_turn_uses_dedicated_filtered_pid_and_timeout(self):
        for definition in (
            "#define TURN_180_SPEED          8.0f",
            "#define TURN_180_KP             2.0f",
            "#define TURN_180_KD             20.0f",
            "#define TURN_180_KI             0.0f",
            "#define TURN_180_D_FILTER       0.2f",
            "#define TURN_180_TIMEOUT_CYCLES 800u",
        ):
            self.assertIn(definition, self.api)

        body = function_body(self.api, "void Chassis_Turn_180_Blocking(void)")
        self.assertIn(
            "gyroT_pid_param.differential_filterK = TURN_180_D_FILTER;", body
        )
        self.assertRegex(
            body,
            r"chassis_turn_blocking\(getAngleZ\(\) \+ 180\.0f,\s*"
            r"TURN_180_DEADBAND,\s*1\);",
        )

        blocking = function_body(
            self.api,
            "static void chassis_turn_blocking(float target_angle, float deadband, uint8_t stage_turn)",
        )
        self.assertIn("TURN_180_TIMEOUT_CYCLES", blocking)

    def test_platform_turn_accumulates_signed_yaw_across_wrap(self):
        body = function_body(self.turn, "uint8_t Stage_turn_Angle(float target)")
        for token in (
            "delta = need2turn(stage_turn_last_yaw, now);",
            "stage_turn_travel += delta;",
            "remaining = TURN_STAGE_TARGET_DEG - stage_turn_travel;",
        ):
            self.assertIn(token, body)

        apply_body = function_body(
            self.turn, "static void stage_turn_apply_speed(float remaining)"
        )
        self.assertIn("gyroT_pid.measure = remaining;", apply_body)
        self.assertIn("gyroT_pid.target = 0.0f;", apply_body)

        def signed_delta(last: float, now: float) -> float:
            delta = now - last
            while delta > 180.0:
                delta -= 360.0
            while delta <= -180.0:
                delta += 360.0
            return delta

        yaw_samples = [-170.0, 170.0, 110.0, 50.0, -10.0]
        travel = sum(
            signed_delta(last, now)
            for last, now in zip(yaw_samples, yaw_samples[1:])
        )
        self.assertAlmostEqual(-200.0, travel)
        self.assertAlmostEqual(20.0, -180.0 - travel)
        self.assertGreater(-180.0 - (-184.0), 0.0)

    def test_platform_turn_tapers_speed_and_skips_common_deadzone(self):
        body = function_body(self.turn, "uint8_t Stage_turn_Angle(float target)")
        for token in (
            "TURN_STAGE_SPEED_FAR",
            "TURN_STAGE_SPEED_MID",
            "TURN_STAGE_SPEED_NEAR",
            "TURN_STAGE_FAR_DEG",
            "TURN_STAGE_MID_DEG",
        ):
            self.assertIn(token, self.turn)
        self.assertIn("stage_turn_apply_speed(remaining);", body)
        self.assertNotIn("turn_deadzone_comp", body)

    def test_platform_turn_requires_stable_angle_and_low_yaw_rate(self):
        body = function_body(self.turn, "uint8_t Stage_turn_Angle(float target)")
        for token in (
            "fabsf(remaining) <= TURN_STAGE_DONE_DEG",
            "fabsf(delta) <= TURN_STAGE_STILL_DEG",
            "stage_turn_stable_count >= TURN_STAGE_STABLE_SAMPLES",
        ):
            self.assertIn(token, body)
        self.assertIn("#define TURN_STAGE_STABLE_SAMPLES 20u", self.turn)

    def test_p2_keeps_generic_turn_api(self):
        p2_start = self.barrier.index("void Stage_P2(void)")
        bridge_start = self.barrier.index("void Barrier_Bridge(void)")
        p2_body = self.barrier[p2_start:bridge_start]
        self.assertIn("Chassis_Turn_By_StopGyro_Blocking(", p2_body)
        self.assertNotIn("Chassis_Turn_180_Blocking(", p2_body)


if __name__ == "__main__":
    unittest.main()
