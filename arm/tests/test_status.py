import contextlib
import io
import json
import subprocess
import sys
import unittest
from unittest.mock import patch

from bucke_arm.arm import FakeArm, Telemetry
from bucke_arm.__main__ import main


class StatusTests(unittest.TestCase):
    def test_default_status_is_labeled_simulated(self):
        output = io.StringIO()
        with patch("bucke_arm.hardware.create_read_only_arm") as hardware:
            with contextlib.redirect_stdout(output):
                self.assertEqual(main(["status"]), 0)
            hardware.assert_not_called()
        self.assertEqual(json.loads(output.getvalue())["source"], "simulated")

    def test_hardware_fails_without_simulated_fallback(self):
        output, error = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(output), contextlib.redirect_stderr(error):
            self.assertEqual(main(["status", "--backend", "hardware"]), 1)
        self.assertEqual(output.getvalue(), "")
        self.assertIn("not implemented", error.getvalue())

    def test_bad_telemetry_is_rejected(self):
        good = (1.0,) * 6
        for bad in ((1.0,) * 5, (float("nan"),) * 6,
                    (float("inf"),) * 6, ("1",) * 6, (True,) * 6):
            for angles, pose in ((bad, good), (good, bad)):
                with self.subTest(angles=angles, pose=pose):
                    with self.assertRaises(ValueError):
                        Telemetry("hardware", angles, pose).validate()

    def test_fake_is_repeatable(self):
        arm = FakeArm()
        self.assertEqual(arm.read_status(), arm.read_status())

    def test_fresh_process_does_not_import_hardware_or_sdk(self):
        code = (
            "import sys; from bucke_arm.__main__ import main; "
            "assert main(['status']) == 0; "
            "assert 'bucke_arm.hardware' not in sys.modules; "
            "assert not any(n == 'pymycobot' or n.startswith('pymycobot.') "
            "for n in sys.modules)"
        )
        result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
