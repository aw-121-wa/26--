"""MaixCam vision bridge for the STM32 explorer_26 firmware.

Protocol peer: App/vision/vision_api.c
Hardware:
    MaixCam A19 UART1_TX -> STM32 PD2 UART5_RX
    MaixCam A18 UART1_RX -> STM32 PC12 UART5_TX
    GND -> GND
"""

try:
    from maix import camera, display, image, nn, time
    from maix.peripheral import uart
    MAIXPY = True
except Exception:
    camera = display = image = nn = time = uart = None
    MAIXPY = False


# ======================== Calibration ========================

CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
DISPLAY_ENABLED = True
DEBUG_OVERLAY = True

UART_DEVICE = "/dev/ttyS1"
UART_BAUDRATE = 115200
UART_READ_CHUNK = 64
HEARTBEAT_INTERVAL_MS = 1000
MAIN_LOOP_SLEEP_MS = 10

# Region used for the traffic sign color card. Tune on the real camera mount.
COLOR_ROI = (55, 35, 210, 155)  # x, y, w, h
COLOR_MIN_PIXELS = 700
COLOR_MIN_AREA = 900

# LAB thresholds used by MaixPy image.find_blobs().
# Tune these under competition lighting. Black threshold is deliberately wide.
COLOR_THRESHOLDS = {
    "black": [(0, 35, -20, 20, -20, 20)],
    "green": [(20, 85, -80, -15, -20, 65)],
    "blue": [(15, 75, -5, 55, -90, -20)],
}

OCR_MODEL_PATH = "/root/models/pp_ocr.mud"
OCR_MIN_CONFIDENCE = 0.45


# ======================== STM32 protocol ========================

SOF0 = 0xAA
SOF1 = 0x55
PROTOCOL_VERSION = 0x01
MAX_PAYLOAD = 16

MSG_HEARTBEAT = 0x01
MSG_RECOGNIZE = 0x11
MSG_CANCEL = 0x12
MSG_RESULT = 0x81
MSG_ERROR = 0x82

MODE_IDLE = 0
MODE_TRAFFIC_LIGHT = 1
MODE_CLUE = 2
MODE_TREASURE = 3

DIRECTION_CENTER = 0
DIRECTION_LEFT = 1
DIRECTION_RIGHT = 2

VALUE_NONE = 0
VALUE_GREEN = 1
VALUE_BLUE_AS_YELLOW = 2
VALUE_BLACK_AS_RED = 3


class ProtocolError(Exception):
    pass


def crc8(data):
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def build_frame(message_type, sequence, payload=b""):
    payload = bytes(payload)
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long")
    body = bytes([PROTOCOL_VERSION, message_type & 0xFF,
                  sequence & 0xFF, len(payload)]) + payload
    return bytes([SOF0, SOF1]) + body + bytes([crc8(body)])


def build_result(sequence, mode, direction, value, confidence):
    payload = bytes([
        mode & 0xFF,
        direction & 0xFF,
        value & 0xFF,
        clamp_u8(confidence, 0, 100),
    ])
    return build_frame(MSG_RESULT, sequence, payload)


def build_error(sequence):
    return build_frame(MSG_ERROR, sequence, b"")


def clamp_u8(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return int(value)


class VisionProtocolParser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        frames = []
        if not data:
            return frames
        self.buffer.extend(data)

        while True:
            if len(self.buffer) < 2:
                return frames

            if self.buffer[0] != SOF0 or self.buffer[1] != SOF1:
                del self.buffer[0]
                continue

            if len(self.buffer) < 7:
                return frames

            version = self.buffer[2]
            length = self.buffer[5]
            frame_len = 2 + 4 + length + 1
            if length > MAX_PAYLOAD:
                del self.buffer[0]
                continue
            if len(self.buffer) < frame_len:
                return frames

            raw = bytes(self.buffer[:frame_len])
            del self.buffer[:frame_len]

            body = raw[2:-1]
            if version != PROTOCOL_VERSION or crc8(body) != raw[-1]:
                continue
            frames.append({
                "type": raw[3],
                "sequence": raw[4],
                "payload": raw[6:-1],
            })


# ======================== Recognition ========================

TEXT_TARGETS = (
    (("东岳泰山", "泰山", "东岳"), 1),
    (("西岳华山", "华山", "西岳"), 2),
    (("南岳衡山", "衡山", "南岳"), 3),
    (("北岳恒山", "恒山", "北岳"), 4),
    (("中岳嵩山", "嵩山", "中岳"), 5),
    (("2号平台", "二号平台", "平台2", "P2"), 2),
    (("3号平台", "三号平台", "平台3", "P3"), 3),
    (("4号平台", "四号平台", "平台4", "P4"), 4),
    (("5号平台", "五号平台", "平台5", "P5"), 5),
    (("6号平台", "六号平台", "平台6", "P6"), 6),
    (("7号平台", "七号平台", "平台7", "P7"), 7),
    (("8号平台", "八号平台", "平台8", "P8"), 8),
)


def normalize_text(text):
    if text is None:
        return ""
    cleaned = str(text).upper()
    for token in (" ", "\t", "\r", "\n", "：", ":", "-", "_", "（", "）", "(", ")"):
        cleaned = cleaned.replace(token, "")
    return cleaned


def map_ocr_text(text):
    cleaned = normalize_text(text)
    if not cleaned:
        return VALUE_NONE
    for aliases, value in TEXT_TARGETS:
        for alias in aliases:
            if normalize_text(alias) in cleaned:
                return value
    if len(cleaned) == 1 and cleaned in "2345678":
        return int(cleaned)
    return VALUE_NONE


def confidence_from_blob(blob, roi):
    try:
        pixels = blob.pixels()
        area = blob.area()
    except Exception:
        return 0
    roi_area = max(1, roi[2] * roi[3])
    pixel_score = min(100, int(pixels * 100 / max(COLOR_MIN_PIXELS, 1)))
    area_score = min(100, int(area * 100 / max(roi_area // 5, 1)))
    return max(0, min(100, (pixel_score + area_score) // 2))


def best_blob(img, thresholds, roi):
    blobs = img.find_blobs(thresholds, roi=roi, pixels_threshold=COLOR_MIN_PIXELS,
                           area_threshold=COLOR_MIN_AREA, merge=True)
    if not blobs:
        return None
    return max(blobs, key=lambda b: b.pixels())


def detect_traffic_color(img):
    candidates = []
    for name, thresholds in COLOR_THRESHOLDS.items():
        blob = best_blob(img, thresholds, COLOR_ROI)
        if blob is None:
            continue
        candidates.append((confidence_from_blob(blob, COLOR_ROI), name, blob))

    if not candidates:
        return VALUE_NONE, 0, None

    confidence, name, blob = max(candidates, key=lambda item: item[0])
    if name == "green":
        return VALUE_GREEN, confidence, blob
    if name == "blue":
        return VALUE_BLUE_AS_YELLOW, confidence, blob
    if name == "black":
        return VALUE_BLACK_AS_RED, confidence, blob
    return VALUE_NONE, 0, blob


def extract_ocr_items(ocr_result):
    if not ocr_result:
        return []
    items = []
    for item in ocr_result:
        text = ""
        score = 1.0
        box = None
        if isinstance(item, dict):
            text = item.get("text") or item.get("label") or item.get("char") or ""
            score = item.get("score", item.get("confidence", 1.0))
            box = item.get("box") or item.get("points")
        elif isinstance(item, (list, tuple)):
            # MaixPy OCR commonly returns tuples/lists containing box, text, score.
            for part in item:
                if isinstance(part, str):
                    text = part
                elif isinstance(part, (int, float)) and 0 <= part <= 1.0:
                    score = float(part)
                elif isinstance(part, (list, tuple)):
                    box = part
        else:
            text = str(item)
        items.append((text, float(score), box))
    return items


class VisionRecognizer:
    def __init__(self):
        if not MAIXPY:
            raise RuntimeError("MaixPy modules are not available")
        self.cam = camera.Camera(CAMERA_WIDTH, CAMERA_HEIGHT)
        self.disp = display.Display() if DISPLAY_ENABLED else None
        self.ocr = None
        try:
            self.ocr = nn.PP_OCR(OCR_MODEL_PATH)
        except Exception as exc:
            print("OCR disabled:", exc)

    def snapshot(self):
        return self.cam.read()

    def recognize(self, mode, direction):
        img = self.snapshot()
        if mode == MODE_TRAFFIC_LIGHT:
            value, confidence, blob = detect_traffic_color(img)
            self.draw_debug(img, mode, value, confidence, blob, [])
            return value, confidence
        if mode in (MODE_CLUE, MODE_TREASURE):
            value, confidence, items = self.detect_text(img)
            self.draw_debug(img, mode, value, confidence, None, items)
            return value, confidence
        self.draw_debug(img, mode, VALUE_NONE, 0, None, [])
        return VALUE_NONE, 0

    def detect_text(self, img):
        if self.ocr is None:
            return VALUE_NONE, 0, []
        try:
            result = self.ocr.detect(img)
        except Exception as exc:
            print("OCR detect failed:", exc)
            return VALUE_NONE, 0, []

        best_value = VALUE_NONE
        best_confidence = 0
        items = extract_ocr_items(result)
        for text, score, _box in items:
            if score < OCR_MIN_CONFIDENCE:
                continue
            value = map_ocr_text(text)
            if value != VALUE_NONE:
                confidence = clamp_u8(int(score * 100), 0, 100)
                if confidence > best_confidence:
                    best_value = value
                    best_confidence = confidence
        return best_value, best_confidence, items

    def draw_debug(self, img, mode, value, confidence, blob, ocr_items):
        if not DEBUG_OVERLAY:
            if self.disp:
                self.disp.show(img)
            return
        try:
            if mode == MODE_TRAFFIC_LIGHT:
                img.draw_rect(COLOR_ROI[0], COLOR_ROI[1], COLOR_ROI[2], COLOR_ROI[3],
                              image.COLOR_YELLOW)
                if blob is not None:
                    img.draw_rect(blob.x(), blob.y(), blob.w(), blob.h(), image.COLOR_RED)
            for text, _score, box in ocr_items:
                if box:
                    x = int(box[0][0]) if isinstance(box[0], (list, tuple)) else 0
                    y = int(box[0][1]) if isinstance(box[0], (list, tuple)) else 0
                    img.draw_string(x, y, text, image.COLOR_GREEN)
            img.draw_string(4, 4, "m:{} v:{} c:{}".format(mode, value, confidence),
                            image.COLOR_WHITE)
        except Exception:
            pass
        if self.disp:
            self.disp.show(img)


class VisionApp:
    def __init__(self):
        if not MAIXPY:
            raise RuntimeError("This app must run on MaixCam/MaixPy")
        self.serial = uart.UART(UART_DEVICE, UART_BAUDRATE)
        self.parser = VisionProtocolParser()
        self.recognizer = VisionRecognizer()
        self.last_heartbeat = 0

    def send(self, frame):
        self.serial.write(frame)

    def poll_uart(self):
        data = self.serial.read(UART_READ_CHUNK)
        if data:
            for frame in self.parser.feed(data):
                self.handle_frame(frame)

    def handle_frame(self, frame):
        message_type = frame["type"]
        sequence = frame["sequence"]
        payload = frame["payload"]

        if message_type == MSG_CANCEL:
            return
        if message_type != MSG_RECOGNIZE or len(payload) != 3:
            self.send(build_error(sequence))
            return

        mode = payload[0]
        direction = payload[1]
        if mode not in (MODE_TRAFFIC_LIGHT, MODE_CLUE, MODE_TREASURE) or direction > DIRECTION_RIGHT:
            self.send(build_error(sequence))
            return

        value, confidence = self.recognizer.recognize(mode, direction)
        self.send(build_result(sequence, mode, direction, value, confidence))

    def heartbeat(self):
        now = ticks_ms()
        if now - self.last_heartbeat >= HEARTBEAT_INTERVAL_MS:
            self.last_heartbeat = now
            self.send(build_frame(MSG_HEARTBEAT, 0, b""))

    def run(self):
        print("MaixCam vision bridge started")
        while True:
            self.poll_uart()
            self.heartbeat()
            sleep_ms(MAIN_LOOP_SLEEP_MS)


def ticks_ms():
    if MAIXPY and time is not None:
        return time.ticks_ms()
    import time as pytime
    return int(pytime.time() * 1000)


def sleep_ms(ms):
    if MAIXPY and time is not None:
        time.sleep_ms(ms)
    else:
        import time as pytime
        pytime.sleep(ms / 1000.0)


def _selftest():
    assert crc8(bytes([PROTOCOL_VERSION, MSG_RECOGNIZE, 7, 3,
                       MODE_TRAFFIC_LIGHT, DIRECTION_LEFT, 1])) == 0x02

    request = build_frame(MSG_RECOGNIZE, 7,
                          bytes([MODE_TRAFFIC_LIGHT, DIRECTION_LEFT, 1]))
    parser = VisionProtocolParser()
    frames = parser.feed(b"\x00\xFF" + request[:3])
    assert frames == []
    frames = parser.feed(request[3:])
    assert len(frames) == 1
    assert frames[0]["type"] == MSG_RECOGNIZE
    assert frames[0]["sequence"] == 7
    assert frames[0]["payload"] == bytes([MODE_TRAFFIC_LIGHT, DIRECTION_LEFT, 1])

    result = build_result(7, MODE_TRAFFIC_LIGHT, DIRECTION_LEFT,
                          VALUE_GREEN, 88)
    assert result == bytes([0xAA, 0x55, 0x01, 0x81, 0x07, 0x04,
                            0x01, 0x01, 0x01, 0x58, 0xF1])
    assert map_ocr_text("到达 东岳泰山") == 1
    assert map_ocr_text("到达7号平台") == 7
    assert map_ocr_text("未知景点") == 0
    print("vision.py selftest passed")


if __name__ == "__main__":
    import sys
    if "--selftest" in sys.argv:
        _selftest()
    else:
        VisionApp().run()
