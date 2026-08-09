from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]


def read_source(relative_path):
    return (ROOT / relative_path).read_text(encoding="utf-8", errors="ignore")


def function_body(source, name):
    match = re.search(
        rf"\b(?:void|uint8_t|HAL_StatusTypeDef|VisionStatus_t)\s+"
        rf"{re.escape(name)}\s*\([^)]*\)\s*\{{",
        source,
    )
    if match is None:
        raise AssertionError(f"missing function {name}")

    depth = 0
    for index in range(match.end() - 1, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[match.end():index]
    raise AssertionError(f"unterminated function {name}")


class Lsc16ActionContractTest(unittest.TestCase):
    def test_driver_declares_actions_and_sends_lsc16_run_frame_on_uart7(self):
        header = read_source("Driver/lsc16_action.h")
        source = read_source("Driver/lsc16_action.c")

        for token in (
            "LSC16_ACTION_INIT_LIE_DOWN = 0u",
            "LSC16_ACTION_STAND_WAVE_LIE_DOWN = 1u",
            "LSC16_ACTION_STAND_UP = 2u",
            "LSC16_ACTION_CAMERA_RIGHT = 3u",
            "LSC16_ACTION_CAMERA_LEFT = 4u",
            "Lsc16_RunActionGroup",
            "Lsc16_RunActionGroupBlocking",
        ):
            self.assertIn(token, header)

        for token in ("0x55u", "0x05u", "0x06u", "huart7"):
            self.assertIn(token, source)
        self.assertNotIn("huart3", source)
        self.assertRegex(source, r"frame\[[^]]+\]\s*=\s*\(uint8_t\)\(times\s*&\s*0xFFu\)")
        self.assertRegex(source, r"frame\[[^]]+\]\s*=\s*\(uint8_t\)\(times\s*>>\s*8u\)")

    def test_uart7_is_configured_for_lsc16_baud_rate(self):
        usart = read_source("Core/Src/usart.c")
        uart7 = function_body(usart, "MX_UART7_Init")

        self.assertIn("huart7.Instance = UART7;", uart7)
        self.assertIn("huart7.Init.BaudRate = 9600;", uart7)
        self.assertIn("huart7.Init.WordLength = UART_WORDLENGTH_8B;", uart7)
        self.assertIn("huart7.Init.StopBits = UART_STOPBITS_1;", uart7)
        self.assertIn("huart7.Init.Parity = UART_PARITY_NONE;", uart7)

    def test_actions_are_hooked_to_start_stage_and_vision_flow(self):
        barrier = read_source("App/barrier/barrier.c")
        vision = read_source("App/vision/vision_api.c")

        self.assertIn('#include "lsc16_action.h"', barrier)
        self.assertIn('#include "lsc16_action.h"', vision)

        zhunbei = function_body(barrier, "zhunbei")
        self.assertLess(
            zhunbei.index("Lsc16_RunActionGroupBlocking(LSC16_ACTION_INIT_LIE_DOWN"),
            zhunbei.index("while (Infrared_ahead == 0)"),
        )
        self.assertLess(
            zhunbei.index("while (Infrared_ahead == 1)"),
            zhunbei.index("Lsc16_RunActionGroupBlocking(LSC16_ACTION_STAND_UP"),
        )

        stage = function_body(barrier, "Stage")
        self.assertLess(
            stage.index("Lsc16_RunActionGroupBlocking(LSC16_ACTION_STAND_WAVE_LIE_DOWN"),
            stage.index("Chassis_Turn_180_Blocking();"),
        )

        stage_p2 = function_body(barrier, "Stage_P2")
        self.assertLess(
            stage_p2.index("Lsc16_RunActionGroupBlocking(LSC16_ACTION_STAND_WAVE_LIE_DOWN"),
            stage_p2.index("Chassis_Turn_By_StopGyro_Blocking"),
        )

        request = function_body(vision, "Vision_Request")
        self.assertIn("direction == VISION_DIRECTION_RIGHT", request)
        self.assertIn("LSC16_ACTION_CAMERA_RIGHT", request)
        self.assertIn("direction == VISION_DIRECTION_LEFT", request)
        self.assertIn("LSC16_ACTION_CAMERA_LEFT", request)

    def test_removed_i2c_and_legacy_rudder_sources_are_not_referenced(self):
        self.assertFalse((ROOT / "Task" / "iic.c").exists())
        self.assertFalse((ROOT / "Task" / "iic.h").exists())

        cmake = read_source("CMakeLists.txt")
        project = read_source("MDK-ARM/explorer_26.uvprojx")
        config = read_source("Driver/config.h")

        for token in ("Driver/uart.c", "rudder_control.c", "Task\\iic.c", "Task/iic.c"):
            self.assertNotIn(token, cmake)
            self.assertNotIn(token, project)
        self.assertIn("lsc16_action.c", project)
        self.assertNotIn("SERVO_NEUTRAL_PULSE", config)


if __name__ == "__main__":
    unittest.main()
