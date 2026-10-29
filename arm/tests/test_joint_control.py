"""Manual R/S selections execute without a second prompt; no real hardware."""

import contextlib
import io
import unittest
from unittest.mock import Mock, patch

from bucke_arm import joint_control as control


def client():
    arm = Mock()
    arm.get_angles.return_value = control.STANDBY_ANGLES.copy()
    arm.is_moving.return_value = 0
    arm.get_error_information.return_value = 0
    arm.get_joint_min_angle.return_value = -180
    arm.get_joint_max_angle.return_value = 180
    return arm


class JointControlTests(unittest.TestCase):
    def run_menu(self, arm, answers):
        sdk = Mock()
        sdk.MyCobot280.return_value = arm
        with patch.dict("sys.modules", {"pymycobot": sdk}), \
                patch("builtins.input", side_effect=answers), \
                contextlib.redirect_stdout(io.StringIO()):
            control.main([])

    def test_r_and_s_dispatch_without_consuming_confirmation_input(self):
        arm = client()
        with patch.object(control, "move_to_ready") as ready, \
                patch.object(control, "move_to_standby") as standby:
            self.run_menu(arm, ["R", "S", "quit"])
        ready.assert_called_once_with()
        standby.assert_called_once_with()
        arm._serial_port.close.assert_called_once_with()

    def test_both_transitions_send_without_prompt_and_verify_completion(self):
        for method, start, target in ((control.move_to_ready, control.STANDBY_ANGLES, control.READY_ANGLES),
                                      (control.move_to_standby, control.READY_ANGLES, control.STANDBY_ANGLES)):
            with self.subTest(method=method.__name__):
                arm = client()
                arm.get_angles.side_effect = [start, start, target, target, target]
                with patch.object(control, "mc", arm), \
                        patch("builtins.input", side_effect=AssertionError("Unexpected confirmation prompt")), \
                        patch.object(control.time, "sleep"), \
                        contextlib.redirect_stdout(io.StringIO()):
                    self.assertTrue(method())
                arm.sync_send_angles.assert_called_once_with(target, control.SYNC_SPEED, timeout=control.SYNC_TIMEOUT)

    def test_wrong_start_still_blocks_transition(self):
        arm = client()
        arm.get_angles.return_value = [20] * 6
        with patch.object(control, "mc", arm), contextlib.redirect_stdout(io.StringIO()):
            self.assertFalse(control.move_to_ready())
        arm.sync_send_angles.assert_not_called()

    def test_individual_joint_still_requires_move_confirmation(self):
        arm = client()
        self.run_menu(arm, ["1", "25", "cancel", "quit"])
        arm.send_angle.assert_not_called()
        self.run_menu(arm, ["1", "25", "MOVE", "quit"])
        arm.send_angle.assert_called_once_with(1, 25, control.SPEED)

    def test_interrupt_stops_and_closes(self):
        arm = client()
        self.run_menu(arm, [KeyboardInterrupt()])
        arm.stop.assert_called_once_with()
        arm._serial_port.close.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
