from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "App" / "vision" / "vision_api.h"


def macro(name: str) -> int:
    text = HEADER.read_text(encoding="utf-8")
    match = re.search(rf"^#define\s+{name}\s+(0x[0-9A-Fa-f]+|\d+)", text, re.MULTILINE)
    assert match, f"missing macro {name}"
    return int(match.group(1), 0)


def crc8_atm(data: bytes) -> int:
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


class VisionProtocolTest(unittest.TestCase):
    def test_protocol_constants_and_crc_vector(self):
        self.assertEqual(macro("VISION_SOF_0"), 0xAA)
        self.assertEqual(macro("VISION_SOF_1"), 0x55)
        self.assertEqual(macro("VISION_PROTOCOL_VERSION"), 0x01)
        self.assertEqual(macro("VISION_MAX_PAYLOAD"), 16)
        self.assertEqual(macro("VISION_INJECT_QUEUE_SIZE"), 8)
        frame_body = bytes([0x01, 0x11, 0x2A, 0x03, 0x01, 0x02, 0x03])
        self.assertEqual(crc8_atm(frame_body), 0xA3)

    def test_protocol_declares_required_messages_and_api(self):
        text = HEADER.read_text(encoding="utf-8")
        for token in (
            "VISION_MSG_HEARTBEAT",
            "VISION_MSG_SET_MODE",
            "VISION_MSG_RECOGNIZE",
            "VISION_MSG_CANCEL",
            "VISION_MSG_ACK",
            "VISION_MSG_RESULT",
            "VISION_MSG_ERROR",
            "Vision_Init",
            "Vision_Poll",
            "Vision_Request",
            "Vision_WaitResult",
            "Vision_TakeResult",
            "Vision_InjectResult",
            "Vision_ScanTrafficPair",
        ):
            self.assertIn(token, text)


if __name__ == "__main__":
    unittest.main()
