from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
VISION_API = ROOT / "App" / "vision" / "vision_api.c"


def _function_body(name):
    text = VISION_API.read_text(encoding="utf-8")
    start = text.index(f"VisionStatus_t {name}(")
    brace = text.index("{", start)
    depth = 0
    for index in range(brace, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
            if depth == 0:
                return text[brace:index + 1]
    raise AssertionError(f"function body not found: {name}")


def test_traffic_pair_scan_turns_right_then_left_then_center():
    body = _function_body("Vision_ScanTrafficPair")

    right_turn = body.index("LSC16_ACTION_CAMERA_RIGHT")
    right_scan = body.index("scan_side(VISION_DIRECTION_RIGHT")
    left_turn = body.index("LSC16_ACTION_CAMERA_LEFT")
    left_scan = body.index("scan_side(VISION_DIRECTION_LEFT")
    center_turn = body.index("LSC16_ACTION_CAMERA_CENTER")

    assert right_turn < right_scan < left_turn < left_scan < center_turn


def test_traffic_pair_scan_failure_does_not_lock_chassis():
    body = _function_body("Vision_ScanTrafficPair")

    assert "Chassis_ForceStop" not in body


if __name__ == "__main__":
    test_traffic_pair_scan_turns_right_then_left_then_center()
    test_traffic_pair_scan_failure_does_not_lock_chassis()
