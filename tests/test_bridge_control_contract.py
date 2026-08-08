from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "App" / "barrier" / "barrier.c"


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


class BridgeControlContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = SOURCE.read_text(encoding="utf-8")
        cls.bridge = function_body(cls.source, "BarrierResult_t Barrier_Bridge(void)")

    def test_bridge_keeps_reference_heading_bias_and_red_correction(self):
        self.assertRegex(
            self.source,
            r"#define\s+BRIDGE_RIGHT_BIAS\s+1\.0f",
        )
        self.assertRegex(
            self.source,
            r"#define\s+BRIDGE_RED_ANGLE\s+2\.0f",
        )
        self.assertEqual(
            len(re.findall(r"gyroG_pid_param\.kp\s*=\s*state->saved_kp\s*\*\s*1\.8f", self.source)),
            2,
        )

    def test_bridge_has_no_pre_or_post_bridge_pause(self):
        self.assertNotIn("vTaskDelay(800)", self.bridge)
        self.assertNotIn("vTaskDelay(300)", self.bridge)
        self.assertIn("Chassis_Ramp_Timeout", self.bridge)


if __name__ == "__main__":
    unittest.main()
