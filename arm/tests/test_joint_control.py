"""The manual menu delegates arm behavior to RealArm/FakeArm."""

import contextlib
import io
import unittest
from unittest.mock import Mock, patch

from bucke_arm import joint_control


class JointControlTests(unittest.TestCase):
    def run_menu(self, arm, answers):
        with patch.object(joint_control, "RealArm", return_value=arm), \
                patch("builtins.input", side_effect=answers), \
                contextlib.redirect_stdout(io.StringIO()):
            joint_control.main([])

    def test_saved_pose_commands_delegate_once_and_close(self):
        arm = Mock()
        self.run_menu(arm, ["R", "READY", "S", "STANDBY", "quit"])
        arm.move_to_ready.assert_called_once_with()
        arm.move_to_standby.assert_called_once_with()
        arm.close.assert_called_once_with()

    def test_cancellation_does_not_move(self):
        arm = Mock()
        self.run_menu(arm, ["R", "cancel", "S", "cancel", "quit"])
        arm.move_to_ready.assert_not_called()
        arm.move_to_standby.assert_not_called()

    def test_manual_joint_and_gripper_commands(self):
        arm = Mock()
        self.run_menu(arm, ["1", "25", "MOVE", "O", "C", "quit"])
        arm.move_joint.assert_called_once_with(1, 25)
        arm.open_gripper.assert_called_once_with()
        arm.close_gripper.assert_called_once_with()

    def test_interrupt_stops_and_closes(self):
        arm = Mock()
        self.run_menu(arm, [KeyboardInterrupt()])
        arm.stop.assert_called_once_with()
        arm.close.assert_called_once_with()

    def test_fake_menu_never_constructs_real_arm(self):
        with patch.object(joint_control, "RealArm") as real, \
                patch("builtins.input", side_effect=["R", "READY", "S", "STANDBY", "quit"]), \
                contextlib.redirect_stdout(io.StringIO()):
            joint_control.main(["--fake"])
        real.assert_not_called()


if __name__ == "__main__":
    unittest.main()
