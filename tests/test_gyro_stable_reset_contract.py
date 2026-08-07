from math import atan2, cos, degrees, hypot, radians, sin
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


def normalize(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle <= -180.0:
        angle += 360.0
    return angle


def circular_mean(samples: list[float], fallback: float) -> float:
    sum_sin = sum(sin(radians(sample)) for sample in samples)
    sum_cos = sum(cos(radians(sample)) for sample in samples)
    if hypot(sum_sin, sum_cos) < 0.001:
        return fallback
    return normalize(degrees(atan2(sum_sin, sum_cos)))


class GyroStableResetContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.chassis = (ROOT / "App" / "chassis" / "chassis_api.c").read_text(
            encoding="utf-8"
        )
        cls.barrier = (ROOT / "App" / "barrier" / "barrier.c").read_text(
            encoding="utf-8"
        )

    def test_crossing_180_uses_circular_mean(self):
        result = circular_mean([179.0, -179.0, 180.0, -180.0], fallback=0.0)
        self.assertGreater(abs(result), 170.0)

        start = self.chassis.index("void GyroStableReset(uint8_t samples, float *angle_out)")
        end = self.chassis.index("uint8_t Stage_DetectedRamp", start)
        body = self.chassis[start:end]
        for token in ("sinf(", "cosf(", "atan2f(", "hypotf("):
            self.assertIn(token, body)

    def test_normal_heading_stays_near_zero(self):
        result = circular_mean([-1.0, 0.0, 1.0], fallback=90.0)
        self.assertAlmostEqual(0.0, result, places=3)

    def test_ambiguous_samples_use_last_valid_fallback(self):
        self.assertEqual(42.0, circular_mean([0.0, 180.0], fallback=42.0))

    def test_platform_keeps_line_approach_then_gyro_ascent(self):
        stage_start = self.barrier.index("void Stage(void)")
        p2_start = self.barrier.index("void Stage_P2(void)")
        body = self.barrier[stage_start:p2_start]
        self.assertIn("Chassis_MotorControl(is_Line", body)
        self.assertIn("RampCtrl_Blocking(RAMP_ASCEND", body)


if __name__ == "__main__":
    unittest.main()
