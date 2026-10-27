"""Behavioral checks for offline ready preview, with no physical arm."""

import contextlib
import io
import json
import subprocess
import sys
import unittest

from bucke_arm.__main__ import main
from bucke_arm.ready import READY_ANGLES_DEG, move_to_ready


class ReadyTests(unittest.TestCase):
    def preview(self, angles):
        return move_to_ready(angles, moving=0, error_information=0)

    def test_corrected_target_and_signed_changes_do_not_mutate_input(self):
        angles = [166.72, .7, -.52, -.79, 89.12, -134.12]
        original = angles.copy()
        result = self.preview(angles)
        self.assertEqual(result.target_angles_deg, (-90, 0, 0, 0, 90, -135))
        self.assertAlmostEqual(result.joint_changes[0].delta_deg, -256.72)
        self.assertAlmostEqual(result.joint_changes[4].delta_deg, .88)
        self.assertAlmostEqual(result.joint_changes[5].delta_deg, -.88)
        self.assertEqual(angles, original)
        self.assertEqual(result.mode, "preview_only")
        self.assertFalse(result.hardware_execution_enabled)
        self.assertFalse(result.route_reviewed)
        self.assertFalse(result.joint_limits_verified)

    def test_j6_positive_to_negative_uses_long_signed_difference(self):
        result = self.preview([-90, 0, 0, 0, 90, 180])
        self.assertEqual(result.joint_changes[5].delta_deg, -315)

    def test_exactly_ready_has_no_changes_but_no_physical_accuracy_claim(self):
        result = self.preview(READY_ANGLES_DEG)
        self.assertTrue(result.exactly_at_target)
        self.assertTrue(all(change.delta_deg == 0 for change in result.joint_changes))
        near = self.preview([-90, 0, 0, 0, 90, -134.9])
        self.assertFalse(near.exactly_at_target)

    def test_moving_and_controller_fault_inputs_are_refused(self):
        for moving, error, message in [(1, 0, "moving"), (0, 5, "Controller error 5")]:
            with self.subTest(moving=moving, error=error):
                with self.assertRaisesRegex(ValueError, message):
                    move_to_ready(READY_ANGLES_DEG, moving=moving, error_information=error)

    def test_invalid_or_missing_status_is_not_assumed_healthy(self):
        for moving, error in [(False, 0), (0, False), (-1, 0), (0, -1), (0, "0")]:
            with self.subTest(moving=moving, error=error), self.assertRaises(ValueError):
                move_to_ready(READY_ANGLES_DEG, moving=moving, error_information=error)
        with self.assertRaises(TypeError):
            move_to_ready(READY_ANGLES_DEG)

    def test_malformed_angles_cannot_produce_a_preview(self):
        cases = [None, "000000", [0] * 5, [0] * 7, [True] * 6,
                 ["0"] * 6, [float("nan")] * 6, [float("inf")] * 6]
        for angles in cases:
            with self.subTest(angles=angles), self.assertRaises(ValueError):
                self.preview(angles)

    def test_cli_rejects_fault_without_partial_plan(self):
        output, errors = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(output), contextlib.redirect_stderr(errors):
            result = main(["move-to-ready", "--angles", "0", "0", "0", "0", "0", "0",
                           "--moving", "0", "--error", "5"])
        self.assertEqual(result, 1)
        self.assertEqual(output.getvalue(), "")
        self.assertIn("Controller error 5", errors.getvalue())

    def test_cli_has_no_hardware_backend_or_implicit_health_status(self):
        base = ["move-to-ready", "--angles", "0", "0", "0", "0", "0", "0"]
        for args in [base, base + ["--moving", "0", "--error", "0", "--backend", "hardware"]]:
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                main(args)
            self.assertEqual(error.exception.code, 2)

    def test_fresh_process_preview_never_imports_hardware_or_sdk(self):
        code = (
            "import sys; from bucke_arm.__main__ import main; "
            "assert main(['move-to-ready', '--angles', '-90', '0', '0', '0', '90', '-135', "
            "'--moving', '0', '--error', '0']) == 0; "
            "assert 'bucke_arm.hardware' not in sys.modules; "
            "assert not any(n == 'pymycobot' or n.startswith('pymycobot.') for n in sys.modules)"
        )
        result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        preview = json.loads(result.stdout)
        self.assertTrue(preview["exactly_at_target"])
        self.assertFalse(preview["hardware_execution_enabled"])


if __name__ == "__main__":
    unittest.main()
