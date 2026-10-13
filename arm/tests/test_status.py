# Standard-library tools let us capture output, replace dependencies, and
# run checks in a separate Python process without installing test packages.
import contextlib
import io
import json
import subprocess
import sys
import unittest
from unittest.mock import patch

from bucke_arm.arm import FakeArm, Telemetry
from bucke_arm.__main__ import main


# unittest discovers methods beginning with test_ and reports pass/fail.
class StatusTests(unittest.TestCase):
    def test_default_status_is_labeled_simulated(self):
        # StringIO collects printed text in memory rather than the terminal.
        output = io.StringIO()
        # Replace the hardware factory with a mock that records any calls.
        with patch("bucke_arm.hardware.create_read_only_arm") as hardware:
            with contextlib.redirect_stdout(output):
                self.assertEqual(main(["status"]), 0)
            # Fake mode must never ask for a physical arm connection.
            hardware.assert_not_called()
        self.assertEqual(json.loads(output.getvalue())["source"], "simulated")

    def test_hardware_fails_without_simulated_fallback(self):
        # Capture normal output and errors separately. An unavailable real
        # backend must report failure, not return simulated readings.
        output, error = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(output), contextlib.redirect_stderr(error):
            self.assertEqual(main(["status", "--backend", "hardware"]), 1)
        self.assertEqual(output.getvalue(), "")
        self.assertIn("not implemented", error.getvalue())

    def test_bad_telemetry_is_rejected(self):
        # Multiplying a one-element tuple creates six identical values.
        # Exercise wrong lengths, NaN, infinity, strings, and booleans.
        good = (1.0,) * 6
        for bad in ((1.0,) * 5, (float("nan"),) * 6,
                    (float("inf"),) * 6, ("1",) * 6, (True,) * 6):
            # Check each malformed case in both angles and tool pose.
            for angles, pose in ((bad, good), (good, bad)):
                # subTest identifies the failing case while trying the rest.
                with self.subTest(angles=angles, pose=pose):
                    with self.assertRaises(ValueError):
                        Telemetry("hardware", angles, pose).validate()

    def test_fake_is_repeatable(self):
        # Repeated calls should return identical data for predictable tests.
        arm = FakeArm()
        self.assertEqual(arm.read_status(), arm.read_status())

    def test_fresh_process_does_not_import_hardware_or_sdk(self):
        # Run in a fresh interpreter so earlier tests cannot influence the
        # imported-module list. Import and default status must not load the
        # hardware adapter or any pymycobot module.
        code = (
            "import sys; from bucke_arm.__main__ import main; "
            "assert main(['status']) == 0; "
            "assert 'bucke_arm.hardware' not in sys.modules; "
            "assert not any(n == 'pymycobot' or n.startswith('pymycobot.') "
            "for n in sys.modules)"
        )
        # Use the same Python interpreter as this test runner.
        result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


# Allows this file to run as a standalone unittest entry point too.
if __name__ == "__main__":
    unittest.main()
