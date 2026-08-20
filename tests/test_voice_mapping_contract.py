from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
VOICE_HEADER = ROOT / "Driver" / "voice_module.h"
VOICE_SOURCE = ROOT / "Driver" / "voice_module.c"
BARRIER_SOURCE = ROOT / "App" / "barrier" / "barrier.c"


class VoiceMappingContractTest(unittest.TestCase):
    EXPECTED = {
        "VOICE_INDEX_READY_START": 9,
        "VOICE_INDEX_PLATFORM_P1": 8,
        "VOICE_INDEX_PLATFORM_P2": 7,
        "VOICE_INDEX_PLATFORM_P3": 6,
        "VOICE_INDEX_PLATFORM_P4": 5,
        "VOICE_INDEX_PLATFORM_P5": 4,
        "VOICE_INDEX_PLATFORM_P6": 3,
        "VOICE_INDEX_PLATFORM_P7": 2,
        "VOICE_INDEX_PLATFORM_P8": 1,
    }

    def _macro_value(self, source, name):
        match = re.search(
            rf"^#define\s+{name}\s+(\d+)u?\s*$",
            source,
            re.MULTILINE,
        )
        self.assertIsNotNone(match, f"missing mapping macro: {name}")
        return int(match.group(1))

    def test_ready_and_platform_numbers_match_required_mapping(self):
        header = VOICE_HEADER.read_text(encoding="utf-8")

        for event, expected in self.EXPECTED.items():
            actual = self._macro_value(header, event)
            print(f"{event}: sound={actual}")
            self.assertEqual(expected, actual, event)

    def test_platform_playback_uses_named_mapping(self):
        barrier = BARRIER_SOURCE.read_text(encoding="utf-8")
        voice = VOICE_SOURCE.read_text(encoding="utf-8")
        combined = barrier + "\n" + voice

        self.assertNotRegex(
            combined,
            r"VoiceModule_PlayIndex\s*\(\s*\d+\s*\)",
        )
        for event in self.EXPECTED:
            self.assertIn(event, combined)

    def test_failure_voice_does_not_collide_with_platform_voice(self):
        header = VOICE_HEADER.read_text(encoding="utf-8")
        failure = self._macro_value(header, "VOICE_INDEX_FAIL_END")
        print(f"VOICE_INDEX_FAIL_END: sound={failure}")
        self.assertNotIn(failure, set(self.EXPECTED.values()))


if __name__ == "__main__":
    unittest.main()
