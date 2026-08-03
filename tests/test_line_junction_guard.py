import ctypes
import _ctypes
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "Sensor" / "line_junction_guard.c"


class GuardState(ctypes.Structure):
    _fields_ = [
        ("enabled", ctypes.c_uint8),
        ("active", ctypes.c_uint8),
        ("has_valid_measure", ctypes.c_uint8),
        ("held_measure", ctypes.c_float),
    ]


class GuardOutput(ctypes.Structure):
    _fields_ = [
        ("measure", ctypes.c_float),
        ("sync_pid_history", ctypes.c_uint8),
    ]


class LineJunctionGuardTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        gcc = shutil.which("gcc")
        if gcc is None:
            raise unittest.SkipTest("host gcc is unavailable")

        cls.temp_dir = tempfile.TemporaryDirectory()
        cls.library_path = Path(cls.temp_dir.name) / "line_junction_guard.dll"
        subprocess.run(
            [
                gcc,
                "-std=c11",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-shared",
                "-o",
                str(cls.library_path),
                str(SOURCE),
            ],
            check=True,
            cwd=ROOT,
            capture_output=True,
            text=True,
        )
        cls.library = ctypes.CDLL(str(cls.library_path))
        cls.library.LineJunctionGuard_Reset.argtypes = [ctypes.POINTER(GuardState)]
        cls.library.LineJunctionGuard_SetEnabled.argtypes = [
            ctypes.POINTER(GuardState),
            ctypes.c_uint8,
        ]
        cls.library.LineJunctionGuard_Update.argtypes = [
            ctypes.POINTER(GuardState),
            ctypes.c_float,
            ctypes.c_uint8,
            ctypes.c_uint8,
            ctypes.c_float,
        ]
        cls.library.LineJunctionGuard_Update.restype = GuardOutput

    @classmethod
    def tearDownClass(cls):
        if hasattr(cls, "temp_dir"):
            handle = cls.library._handle
            del cls.library
            if os.name == "nt":
                _ctypes.FreeLibrary(handle)
            else:
                _ctypes.dlclose(handle)
            cls.temp_dir.cleanup()

    def setUp(self):
        self.state = GuardState()
        self.library.LineJunctionGuard_Reset(ctypes.byref(self.state))
        self.library.LineJunctionGuard_SetEnabled(ctypes.byref(self.state), 1)

    def update(self, measure, line_count, led_count, target=0.0):
        return self.library.LineJunctionGuard_Update(
            ctypes.byref(self.state),
            measure,
            line_count,
            led_count,
            target,
        )

    def test_holds_last_valid_measure_through_junction_without_repeated_sync(self):
        normal = self.update(0.6, 1, 2)
        entering = self.update(-2.0, 2, 4)
        staying = self.update(1.5, 3, 6)

        self.assertAlmostEqual(normal.measure, 0.6, places=5)
        self.assertEqual(normal.sync_pid_history, 0)
        self.assertAlmostEqual(entering.measure, 0.6, places=5)
        self.assertEqual(entering.sync_pid_history, 1)
        self.assertAlmostEqual(staying.measure, 0.6, places=5)
        self.assertEqual(staying.sync_pid_history, 0)

    def test_recovery_uses_real_measure_and_requests_bumpless_pid_sync(self):
        self.update(0.4, 1, 2)
        self.update(-1.0, 2, 4)

        recovery = self.update(0.1, 1, 2)
        stable = self.update(0.2, 1, 2)

        self.assertAlmostEqual(recovery.measure, 0.1, places=5)
        self.assertEqual(recovery.sync_pid_history, 1)
        self.assertAlmostEqual(stable.measure, 0.2, places=5)
        self.assertEqual(stable.sync_pid_history, 0)

    def test_junction_without_history_falls_back_to_target(self):
        output = self.update(-2.0, 2, 5, target=0.25)

        self.assertAlmostEqual(output.measure, 0.25, places=5)
        self.assertEqual(output.sync_pid_history, 1)

    def test_disabling_guard_releases_raw_measure_without_derivative_kick(self):
        self.update(0.5, 1, 2)
        self.update(-1.5, 2, 5)
        self.library.LineJunctionGuard_SetEnabled(ctypes.byref(self.state), 0)

        output = self.update(-1.5, 2, 5)

        self.assertAlmostEqual(output.measure, -1.5, places=5)
        self.assertEqual(output.sync_pid_history, 1)


if __name__ == "__main__":
    unittest.main()
