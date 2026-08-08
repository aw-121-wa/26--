from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]


class N12ReferenceContractTest(unittest.TestCase):
    def test_n12_prefix_uses_known_good_26h_edge_parameters(self):
        source = (ROOT / "App" / "map" / "map_message.c").read_text(
            encoding="utf-8"
        )
        expected_edges = (
            r"/\*B2 -> N4\*/\s*\{N4, CLEFT\|MCLEFT\|LEFT_LINE, 140, 6, SPEED2, NONE\}",
            r"\{N3, DLEFT\|Temp_R\|LEFT_LINE, 0, 100, SPEED3, NONE\}",
            r"\{N4, RIGHT_LINE\|Temp_L\|MUL2SING, 0, 120, SPEED3, NONE\}",
            r"\{N5, DLEFT\|RIGHT_LINE, 0, 95, SPEED3, NONE\}",
            r"\{P4, LiuShui\|RIGHT_LINE, 180, 55, SPEED3, UpStage\}",
        )
        for pattern in expected_edges:
            self.assertRegex(source, pattern)


if __name__ == "__main__":
    unittest.main()
