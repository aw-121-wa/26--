from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
BARRIER_SOURCE = ROOT / "App" / "barrier" / "barrier.c"


def zhunbei_body():
    source = BARRIER_SOURCE.read_text(encoding="utf-8")
    start = source.index("void zhunbei(void)")
    end = source.index("/* ======================== 通用平台处理", start)
    return source[start:end]


class ZhunbeiStartContractTest(unittest.TestCase):
    def test_zhunbei_waits_for_barrier_removal_before_ready_voice(self):
        body = zhunbei_body()

        self.assertIn("while (Infrared_ahead == 0)", body)
        self.assertIn("while (Infrared_ahead == 1)", body)
        self.assertLess(
            body.index("while (Infrared_ahead == 1)"),
            body.index("VoiceModule_PlayReadyStart()"),
        )

    def test_zhunbei_ready_voice_precedes_complete_gesture_sequence(self):
        body = zhunbei_body()

        tokens = (
            "VoiceModule_PlayReadyStart()",
            "LSC16_ACTION_STAND_UP",
            "LSC16_ACTION_WAVE_LEFT",
            "LSC16_ACTION_WAVE_RIGHT",
            "LSC16_ACTION_LIE_DOWN",
            "LSC16_ACTION_CAMERA_CENTER",
            "#if LINE_DEBUG_MODE",
        )
        positions = [body.index(token) for token in tokens]
        self.assertEqual(positions, sorted(positions))

    def test_zhunbei_does_not_use_barrier_detected_action(self):
        body = zhunbei_body()
        for old_name in (
            "LSC16_ACTION_INIT_LIE_DOWN",
            "LSC16_ACTION_STAND_WAVE_LIE_DOWN",
            "LSC16_ACTION_BARRIER_DETECTED",
            "LSC16_ACTION_TURN_DONE",
        ):
            self.assertNotIn(old_name, body)

    def test_zhunbei_action_precedes_p2_departure_branches(self):
        body = zhunbei_body()

        action = body.index("LSC16_ACTION_CAMERA_CENTER")
        departure = body.index("#if LINE_DEBUG_MODE")
        self.assertLess(action, departure)

    def test_stage_keeps_unified_platform_flow(self):
        source = BARRIER_SOURCE.read_text(encoding="utf-8")
        stage_start = source.index("void Stage(void)")
        next_section = source.index("/* ======================== 过桥处理", stage_start)
        stage_body = source[stage_start:next_section]

        self.assertIn("barrier_platform_start_gesture()", stage_body)
        self.assertIn("barrier_play_platform_voice()", stage_body)
        self.assertIn("barrier_platform_center()", stage_body)


if __name__ == "__main__":
    unittest.main()
