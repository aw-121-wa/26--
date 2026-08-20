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

    def test_zhunbei_ready_voice_precedes_turn_done_action(self):
        body = zhunbei_body()

        self.assertIn("VoiceModule_PlayReadyStart()", body)
        self.assertIn("LSC16_ACTION_TURN_DONE", body)
        self.assertLess(
            body.index("VoiceModule_PlayReadyStart()"),
            body.index("LSC16_ACTION_TURN_DONE"),
        )

    def test_zhunbei_does_not_use_barrier_detected_action(self):
        self.assertNotIn("LSC16_ACTION_BARRIER_DETECTED", zhunbei_body())

    def test_zhunbei_action_precedes_p2_departure_branches(self):
        body = zhunbei_body()

        action = body.index("LSC16_ACTION_TURN_DONE")
        departure = body.index("#if LINE_DEBUG_MODE")
        self.assertLess(action, departure)

    def test_stage_keeps_generic_barrier_action(self):
        source = BARRIER_SOURCE.read_text(encoding="utf-8")
        stage_start = source.index("void Stage(void)")
        next_section = source.index("/* ======================== 过桥处理", stage_start)

        self.assertIn(
            "barrier_board_detected_action(",
            source[stage_start:next_section],
        )


if __name__ == "__main__":
    unittest.main()
