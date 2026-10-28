"""Offline checks for consent, torque sequencing, and interrupted recovery."""

import contextlib
import io
import unittest
from unittest.mock import Mock, patch

from bucke_arm.recover_joint4 import main, recover, resume_hold


# Historical recovery pose used as repeatable test data, not a new motion target.
START = [43.06, -14.58, 11.07, 153.8, 79.8, -46.0]


def pose(j4):
    # Vary only J4 while keeping a fresh copy of the other joint values.
    values = START[:]
    values[3] = j4
    return values


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        # No motion, calibration, gripper, or all-servo method exists here.
        self.client = Mock(spec=["get_system_version", "is_moving",
                                 "get_error_information", "get_angles",
                                 "release_servo", "focus_servo"])
        self.client.get_system_version.return_value = 7.3
        self.client.is_moving.return_value = 0
        self.client.get_error_information.return_value = 4
        self.client.get_angles.return_value = START
        self.client.release_servo.return_value = 1
        self.client.focus_servo.return_value = 1
        self.out, self.err = io.StringIO(), io.StringIO()

        # Keep these tests compatible with the Pi's Python 3.8.
        for redirect in (contextlib.redirect_stdout(self.out),
                         contextlib.redirect_stderr(self.err)):
            redirect.__enter__()
            self.addCleanup(redirect.__exit__, None, None, None)

    def run_recovery(self, answers):
        # Script operator answers (or injected exceptions) and skip actual pauses.
        return recover(self.client, Mock(side_effect=answers), lambda _: None)

    def test_default_does_not_load_sdk_or_check_packages(self):
        # Preview must work on a laptop without a robot SDK installation.
        with patch("bucke_arm.recover_joint4.metadata.version") as version:
            self.assertEqual(main([]), 0)

        version.assert_not_called()

    def test_cancel_never_changes_torque(self):
        # Lack of exact release consent must leave both torque methods unused.
        self.assertEqual(self.run_recovery(["cancel"]), 0)
        self.client.release_servo.assert_not_called()
        self.client.focus_servo.assert_not_called()

    def test_preconditions_block_release(self):
        # Restore each modified reply so the next subcase tests one fault at a time.
        for method, bad in (("get_system_version", 7.2), ("is_moving", -1),
                            ("get_error_information", 16),
                            ("get_angles", [float("nan")] * 6)):
            with self.subTest(method=method):
                original = getattr(self.client, method).return_value
                getattr(self.client, method).return_value = bad
                with self.assertRaises(RuntimeError):
                    self.run_recovery(["RELEASE J4"])

                getattr(self.client, method).return_value = original

        self.client.release_servo.assert_not_called()

    def test_in_range_pose_requires_new_plan(self):
        # The historical recovery window must not become a general release tool.
        self.client.get_angles.return_value = pose(145)
        with self.assertRaises(RuntimeError):
            self.run_recovery(["RELEASE J4"])

        self.client.release_servo.assert_not_called()

    def test_failed_release_warns_without_automatic_focus(self):
        # A missing acknowledgement cannot prove whether release physically happened.
        self.client.release_servo.return_value = -1
        with self.assertRaises(RuntimeError):
            self.run_recovery(["RELEASE J4"])

        self.client.focus_servo.assert_not_called()
        self.assertIn("KEEP SUPPORTING", self.err.getvalue())

    def test_interrupt_never_automatically_reengages(self):
        # Ctrl+C after release must warn about support instead of focusing in cleanup.
        with self.assertRaises(KeyboardInterrupt):
            self.run_recovery(["RELEASE J4", KeyboardInterrupt()])

        self.client.focus_servo.assert_not_called()
        self.assertIn("KEEP SUPPORTING", self.err.getvalue())

    def test_focus_refused_until_in_recovery_window(self):
        # Replay an out-of-window attempt followed by valid readings; check that
        # the eventual focus follows release and only occurs once.
        self.client.get_angles.side_effect = [START, pose(151), pose(145), pose(145)]
        self.client.get_error_information.side_effect = [4, 0]

        self.assertEqual(self.run_recovery(["RELEASE J4", "HOLD J4", "HOLD J4"]), 0)
        self.client.release_servo.assert_called_once_with(4)
        self.client.focus_servo.assert_called_once_with(4)
        self.assertLess(self.client.method_calls.index(unittest.mock.call.release_servo(4)),
                        self.client.method_calls.index(unittest.mock.call.focus_servo(4)))
        self.assertIn("Re-engagement refused", self.out.getvalue())

    def test_other_joint_shift_aborts_without_focus(self):
        # Movement outside J4 invalidates the original recovery assumptions.
        shifted = pose(145)
        shifted[0] += 3
        self.client.get_angles.side_effect = [START, shifted]
        with self.assertRaises(RuntimeError):
            self.run_recovery(["RELEASE J4", "HOLD J4"])

        self.client.focus_servo.assert_not_called()

    def test_failed_focus_is_not_reported_as_holding(self):
        # Failure to acknowledge holding must leave the support warning active.
        self.client.get_angles.side_effect = [START, pose(145)]
        self.client.focus_servo.return_value = -1
        with self.assertRaises(RuntimeError):
            self.run_recovery(["RELEASE J4", "HOLD J4"])

        self.assertIn("KEEP SUPPORTING", self.err.getvalue())

    def test_low_angle_no_longer_exits_original_session(self):
        # Leaving the original recovery window should direct the operator to
        # resume-hold, without automatically re-engaging at an unreviewed angle.
        self.client.get_angles.side_effect = [START, pose(117.42)]

        self.assertEqual(self.run_recovery(["RELEASE J4", "HOLD J4", "ABORT"]), 1)
        self.client.focus_servo.assert_not_called()
        self.assertIn("--resume-hold", self.out.getvalue())

    def run_resume(self, answers):
        # Resume tests inject the same fake input and no-op pause as recovery tests.
        return resume_hold(self.client, Mock(side_effect=answers), lambda _: None)

    def test_resume_at_screenshot_pose_never_releases(self):
        # Resume can request holding at this historical pose without another release.
        self.client.get_angles.return_value = [42.97, -14.58, 11.07, 117.42, 81.21, -45.7]

        self.assertEqual(self.run_resume(["HOLD J4"]), 0)
        self.client.release_servo.assert_not_called()
        self.client.focus_servo.assert_called_once_with(4)

    def test_resume_cancellation_and_interrupt_never_change_torque(self):
        # Cancellation or Ctrl+C before focus must cause no torque-changing call.
        self.client.get_angles.return_value = pose(117.42)

        self.assertEqual(self.run_resume(["cancel"]), 1)
        with self.assertRaises(KeyboardInterrupt):
            self.run_resume([KeyboardInterrupt()])

        self.client.release_servo.assert_not_called()
        self.client.focus_servo.assert_not_called()
        self.assertIn("KEEP SUPPORTING", self.err.getvalue())

    def test_resume_rejects_invalid_out_of_range_or_unstable_readings(self):
        # Provide different sequences for the initial, post-prompt, and stability
        # readings. Each sequence must fail before any holding request.
        for samples in ([pose(153.8)], [pose(117), None],
                        [pose(117), pose(119), pose(119)],
                        [pose(117), pose(117), pose(149.5)]):
            with self.subTest(samples=samples):
                self.client.get_angles.side_effect = samples
                with self.assertRaises(RuntimeError):
                    self.run_resume(["HOLD J4"])

        self.client.release_servo.assert_not_called()
        self.client.focus_servo.assert_not_called()

    def test_resume_refuses_new_fault_before_focus(self):
        # A new error after consent must be caught by the last pre-focus query.
        self.client.get_angles.return_value = pose(117.42)
        self.client.get_error_information.side_effect = [4, 16]
        with self.assertRaises(RuntimeError):
            self.run_resume(["HOLD J4"])

        self.client.focus_servo.assert_not_called()

    def test_resume_failed_acknowledgement_warns_without_retry(self):
        # Retry could cause unexpected action; require one attempt and a warning.
        self.client.get_angles.return_value = pose(117.42)
        self.client.focus_servo.return_value = -1
        with self.assertRaises(RuntimeError):
            self.run_resume(["HOLD J4"])

        self.client.focus_servo.assert_called_once_with(4)
        self.client.release_servo.assert_not_called()
        self.assertIn("KEEP SUPPORTING", self.err.getvalue())

    def test_resume_position_jump_after_focus_warns(self):
        # Acknowledgement is insufficient if the next position differs excessively.
        self.client.get_angles.side_effect = [pose(117)] * 3 + [pose(123)]
        with self.assertRaises(RuntimeError):
            self.run_resume(["HOLD J4"])

        self.assertIn("KEEP SUPPORTING", self.err.getvalue())

    def test_resume_preview_never_checks_sdk(self):
        # The alternate preview mode must also stay independent of installed SDKs.
        with patch("bucke_arm.recover_joint4.metadata.version") as version:
            self.assertEqual(main(["--resume-hold"]), 0)

        version.assert_not_called()


if __name__ == "__main__":
    unittest.main()
