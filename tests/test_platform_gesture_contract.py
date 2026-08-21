from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
BARRIER_SOURCE = ROOT / "App" / "barrier" / "barrier.c"
ACTION_HEADER = ROOT / "Driver" / "lsc16_action.h"


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


class PlatformGestureContractTest(unittest.TestCase):
    EXPECTED_ACTIONS = {
        "LSC16_ACTION_LIE_DOWN": 0,
        "LSC16_ACTION_STAND_UP": 1,
        "LSC16_ACTION_WAVE_LEFT": 2,
        "LSC16_ACTION_WAVE_RIGHT": 3,
        "LSC16_ACTION_WAVE_STOP": 4,
        "LSC16_ACTION_CAMERA_LEFT": 5,
        "LSC16_ACTION_CAMERA_RIGHT": 6,
        "LSC16_ACTION_CAMERA_CENTER": 7,
    }
    EXPECTED_WAITS = {
        "LSC16_WAIT_STAND_MS": 100,
        "LSC16_WAIT_LIE_MS": 100,
        "LSC16_WAIT_GESTURE_MS": 100,
        "LSC16_WAIT_CAMERA_MS": 100,
    }
    EXPECTED_PLATFORM_DISTANCES = {
        "DISTANCE_PLATFORM_FRONT": 8.0,
        "DISTANCE_P2_BOARD_FRONT": 6.0,
        "SOUTH_POLE_BOARD_FRONT": 10.0,
        "HIGH_MOUNTAIN_BOARD_FRONT": 8.0,
    }

    @classmethod
    def setUpClass(cls):
        cls.barrier = BARRIER_SOURCE.read_text(encoding="utf-8")
        cls.header = ACTION_HEADER.read_text(encoding="utf-8")

    def _enum_value(self, name):
        match = re.search(rf"\b{name}\s*=\s*(\d+)u?\b", self.header)
        self.assertIsNotNone(match, f"missing action enum: {name}")
        return int(match.group(1))

    def _macro_value(self, name):
        match = re.search(rf"^#define\s+{name}\s+(\d+)u?\s*$", self.header, re.MULTILINE)
        self.assertIsNotNone(match, f"missing action wait: {name}")
        return int(match.group(1))

    def _barrier_macro_float_value(self, name):
        match = re.search(
            rf"^#define\s+{name}\s+([0-9]+(?:\.[0-9]+)?)(?:f)?",
            self.barrier,
            re.MULTILINE,
        )
        self.assertIsNotNone(match, f"missing barrier distance: {name}")
        return float(match.group(1))

    def test_action_groups_match_new_lsc16_mapping(self):
        for name, expected in self.EXPECTED_ACTIONS.items():
            self.assertEqual(expected, self._enum_value(name), name)

    def test_old_action_aliases_are_removed_from_business_code(self):
        sources = "\n".join(
            path.read_text(encoding="utf-8")
            for path in (BARRIER_SOURCE, ACTION_HEADER, ROOT / "App" / "vision" / "vision_api.c")
        )
        for old_name in (
            "LSC16_ACTION_INIT_LIE_DOWN",
            "LSC16_ACTION_STAND_WAVE_LIE_DOWN",
            "LSC16_ACTION_BARRIER_DETECTED",
            "LSC16_ACTION_TURN_DONE",
        ):
            self.assertNotIn(old_name, sources)

    def test_action_waits_are_100ms(self):
        for name, expected in self.EXPECTED_WAITS.items():
            self.assertEqual(expected, self._macro_value(name), name)

    def test_platform_forward_distances_are_independent_and_correct(self):
        for name, expected in self.EXPECTED_PLATFORM_DISTANCES.items():
            self.assertEqual(expected, self._barrier_macro_float_value(name), name)
        self.assertNotIn("BARRIER_AFTER_BOARD_FRONT", self.barrier)

    def test_shared_gesture_is_stand_left_right(self):
        body = function_body(self.barrier, "barrier_platform_start_gesture")
        positions = [
            body.index("LSC16_ACTION_STAND_UP"),
            body.index("LSC16_ACTION_WAVE_LEFT"),
            body.index("LSC16_ACTION_WAVE_RIGHT"),
        ]
        self.assertEqual(positions, sorted(positions))

    def test_platforms_use_forward_gesture_reverse_voice_turn_center(self):
        platform_tokens = {
            "void Stage(void)": (
                "Chassis_DriveDistance_Blocking(is_Gyro, DISTANCE_PLATFORM_FRONT",
                "barrier_platform_start_gesture()",
                "Chassis_DriveDistance_Blocking(is_Gyro, DISTANCE_PLATFORM_BACK",
                "barrier_play_platform_voice()",
                "Chassis_Turn_180_Blocking()",
                "barrier_platform_center()",
            ),
            "void Stage_P2(void)": (
                "barrier_wait_front_infrared(",
                "Chassis_DriveDistance_Blocking(is_Gyro, DISTANCE_P2_BOARD_FRONT",
                "barrier_platform_start_gesture()",
                "Chassis_DriveDistance_Blocking(is_Gyro, DISTANCE_PLATFORM_BACK",
                "barrier_play_platform_voice()",
                "Chassis_Turn_180_Blocking()",
                "barrier_platform_center()",
            ),
            "void Barrier_SouthPole(void)": (
                "barrier_wait_front_infrared(",
                "barrier_drive_distance(is_Gyro, SOUTH_POLE_BOARD_FRONT",
                "barrier_platform_start_gesture()",
                "barrier_reverse_distance(5.0f",
                "barrier_play_platform_voice()",
                "Chassis_Turn_180_Blocking()",
                "barrier_platform_center()",
            ),
            "void Barrier_HighMountain(void)": (
                "barrier_wait_front_infrared(",
                "barrier_drive_distance(is_Gyro, HIGH_MOUNTAIN_BOARD_FRONT",
                "barrier_platform_start_gesture()",
                "barrier_reverse_distance(6.0f",
                "barrier_play_platform_voice()",
                "Chassis_Turn_180_Blocking()",
                "barrier_platform_center()",
            ),
        }

        for signature, tokens in platform_tokens.items():
            body = function_body(self.barrier, signature)
            positions = [body.index(token) for token in tokens]
            self.assertEqual(positions, sorted(positions), signature)

    def test_front_infrared_wait_is_guarded(self):
        body = function_body(self.barrier, "barrier_wait_front_infrared")
        for guard in (
            "barrier_distance_exceeded",
            "barrier_wait_expired",
            "Chassis_IsStopLocked",
        ):
            self.assertIn(guard, body)


if __name__ == "__main__":
    unittest.main()
