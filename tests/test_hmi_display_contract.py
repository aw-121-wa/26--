import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read_source(relative_path):
    return (ROOT / relative_path).read_text(encoding="utf-8", errors="ignore")


class HmiDisplayContractTests(unittest.TestCase):
    def test_screen_module_exposes_score_recording_and_uses_uart8(self):
        header = read_source("Driver/hmi_display.h")
        source = read_source("Driver/hmi_display.c")

        for symbol in (
            "HmiDisplay_Init",
            "HmiDisplay_ResetScores",
            "HmiDisplay_RecordArrival",
            "HmiDisplay_Tick",
            "HmiDisplay_GetScores",
        ):
            self.assertIn(symbol, header)
            self.assertIn(symbol, source)

        self.assertIn("#define HMI_DISPLAY_UART huart8", source)
        self.assertIn("#define HMI_HOME_NODE P2", source)
        self.assertIn("HAL_UART_Transmit_IT(&HMI_DISPLAY_UART", source)
        self.assertNotIn('"page 0"', source)
        self.assertRegex(
            source,
            r"case 0u:\s*\(void\)snprintf\(buf, size, \"cls 0\"\);",
        )

    def test_screen_score_rules_match_rule_book_categories(self):
        source = read_source("Driver/hmi_display.c")
        compact = re.sub(r"\s+", " ", source)

        self.assertRegex(source, r"function == View\s*\|\|\s*function == View1")
        self.assertRegex(source, r"upright_spots\+\+;")
        self.assertRegex(source, r"base_score = .*base_score \+ 11u")
        self.assertRegex(compact, r"case P1:.*case P2:.*case P3:.*case P4:.*case P5:")
        self.assertRegex(source, r"platforms_1_to_5\+\+;\s*return 30u;")
        self.assertRegex(compact, r"case P6:.*platform_6\+\+;.*return 30u;")
        self.assertRegex(compact, r"case P7:.*platform_7\+\+;.*return 90u;")
        self.assertRegex(compact, r"case P8:.*platform_8\+\+;.*return 150u;")
        self.assertRegex(source, r"node == HMI_HOME_NODE")
        self.assertRegex(source, r"home_returns\+\+;")
        self.assertRegex(source, r"total_score \+= hmi_round_base_score / 5u;")

    def test_map_and_motor_tasks_are_hooked_to_screen_module(self):
        map_source = read_source("App/map/map.c")
        motor_source = read_source("Task/motor_task.c")

        self.assertIn('#include "hmi_display.h"', map_source)
        self.assertIn("HmiDisplay_ResetScores();", map_source)
        self.assertRegex(
            map_source,
            r"static void cross_node_advance\(void\)\s*\{\s*HmiDisplay_RecordArrival"
            r"\(nodesr\.nowNode\.nodenum, nodesr\.nowNode\.function\);",
        )

        self.assertIn('#include "hmi_display.h"', motor_source)
        self.assertIn("HmiDisplay_Init();", motor_source)
        self.assertRegex(
            motor_source,
            r"debug_uart_tick\(\);\s*HmiDisplay_Tick\(\);",
        )


if __name__ == "__main__":
    unittest.main()
