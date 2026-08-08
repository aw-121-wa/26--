from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class NoDebugSnapshotArtifactsTest(unittest.TestCase):
    def test_debug_snapshot_module_is_absent(self):
        self.assertFalse((ROOT / "Task" / "debug_snapshot.c").exists())
        self.assertFalse((ROOT / "Task" / "debug_snapshot.h").exists())

        production = "\n".join(
            path.read_text(encoding="utf-8", errors="ignore")
            for folder in ("App", "Task", "Core")
            for path in (ROOT / folder).rglob("*.[ch]")
        )
        self.assertNotIn("DebugSnapshot", production)
        self.assertNotIn("debug_snapshot", production)


if __name__ == "__main__":
    unittest.main()
