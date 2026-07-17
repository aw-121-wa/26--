from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]


class RouteContractTest(unittest.TestCase):
    def test_builder_api_and_invalid_index(self):
        header = (ROOT / "App" / "map" / "route_builder.h").read_text(encoding="utf-8")
        for token in (
            "ROUTE_CAPACITY",
            "RouteBuilder_Init",
            "RouteBuilder_Clear",
            "RouteBuilder_AppendNode",
            "RouteBuilder_AppendSegment",
            "RouteBuilder_AppendShortestPath",
            "RouteBuilder_Validate",
            "RouteBuilder_Commit",
        ):
            self.assertIn(token, header)

        map_header = (ROOT / "App" / "map" / "map.h").read_text(encoding="utf-8")
        self.assertIn("MAP_NODE_INDEX_INVALID", map_header)
        self.assertIn("Map_ValidateData", map_header)

    def test_catalog_declares_all_legacy_routes(self):
        text = (ROOT / "App" / "map" / "route_catalog.c").read_text(encoding="utf-8")
        self.assertEqual(len(re.findall(r"^LEGACY_CLUE_ROUTE\(", text, re.MULTILINE)), 80)
        self.assertEqual(len(re.findall(r"^LEGACY_DOOR_ROUTE\(", text, re.MULTILINE)), 14)

    def test_all_catalog_fragments_are_connected(self):
        map_header = (ROOT / "App" / "map" / "map.h").read_text(encoding="utf-8")
        enum_body = re.search(r"enum MapNode\s*\{(.*?)\};", map_header, re.DOTALL).group(1)
        values = {}
        current = -1
        for name, explicit in re.findall(r"\b([A-Z]\w*)\s*(?:=\s*(\d+))?\s*,?", enum_body):
            current = int(explicit) if explicit else current + 1
            values[name] = current

        message = (ROOT / "App" / "map" / "map_message.c").read_text(encoding="utf-8")
        node_body = re.search(r"NODE Node\[126\]\s*=\s*\{(.*?)\n\};", message, re.DOTALL).group(1)
        targets = [values[name] for name in re.findall(r"\{\s*([A-Z]\w*)\s*,", node_body)]
        counts_body = re.search(r"ConnectionNum\[52\].*?\{(.*?)\};", message, re.DOTALL).group(1)
        counts = [int(number) for number in re.findall(r"\d+", counts_body)]
        self.assertEqual(len(targets), 126)
        self.assertEqual(sum(counts), 126)

        adjacency = []
        offset = 0
        for count in counts:
            adjacency.append(set(targets[offset:offset + count]))
            offset += count

        catalog = (ROOT / "App" / "map" / "route_catalog.c").read_text(encoding="utf-8")
        declarations = re.findall(
            r"LEGACY_(?:DOOR|CLUE)_ROUTE\(\d+,\s*([^;]+)\);", catalog
        )
        self.assertEqual(len(declarations), 94)
        for declaration in declarations:
            route_nodes = [
                values[name]
                for name in re.findall(r"\b(?:S|P|N|B|C|G)\d+\b", declaration)
            ]
            for start, target in zip(route_nodes, route_nodes[1:]):
                self.assertIn(target, adjacency[start], f"disconnected {start}->{target}")

        map_source = (ROOT / "App" / "map" / "map.c").read_text(encoding="utf-8")
        initial_route = re.search(r"u8 route\[100\]\s*=\s*\{(.*?)\};", map_source, re.DOTALL).group(1)
        initial_nodes = [
            values[name]
            for name in re.findall(r"\b(?:S|P|N|B|C|G)\d+\b", initial_route)
        ]
        current = values["P2"]
        for target in initial_nodes:
            self.assertIn(target, adjacency[current], f"initial route {current}->{target}")
            current = target

    def test_all_return_route_combinations_have_unique_catalog_slots(self):
        selected = set()
        for group in range(4):
            for green_gate in range(1, 5):
                for treasure_offset in range(5):
                    route_number = group * 20 + (4 - green_gate) * 5 + treasure_offset + 1
                    self.assertGreaterEqual(route_number, 1)
                    self.assertLessEqual(route_number, 80)
                    selected.add(route_number)
        self.assertEqual(selected, set(range(1, 81)))


if __name__ == "__main__":
    unittest.main()
