"""Exercise the manual menu through the same arm methods used by callers."""

import contextlib
import io
import unittest
from unittest.mock import Mock, patch

from bucke_arm import joint_control as control
from bucke_arm.arm import FakeArm, MOVE_TIMEOUT, POSE_SPEED, READY_ANGLES, RealArm, STANDBY_ANGLES


def client_at_standby():
    client = Mock()
    client.get_angles.return_value = STANDBY_ANGLES
    client.is_moving.return_value = 0
    client.get_error_information.return_value = 0
    client.get_joint_min_angle.return_value = -180
    client.get_joint_max_angle.return_value = 180
    return client


class JointControlTests(unittest.TestCase):
    def run_menu(self, client, answers):
        with patch.object(control, "RealArm", return_value=RealArm(client)), \
                patch("bucke_arm.arm.time.sleep"), \
                patch("builtins.input", side_effect=answers), \
                contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(control.main([]), 0)
        client._serial_port.close.assert_called_once_with()
        return output.getvalue()

    def test_r_and_s_execute_and_confirm_arrival_without_extra_input(self):
        client = client_at_standby()
        client.get_angles.side_effect = (
            [STANDBY_ANGLES] * 2 + [READY_ANGLES] * 5 + [STANDBY_ANGLES] * 3
        )
        output = self.run_menu(client, ["V", "25", "R", "S", "Q"])
        for pose in ("READY", "STANDBY"):
            self.assertIn(f"--- {pose} TRANSITION ---", output)
            self.assertIn(f"{pose} POSITION CONFIRMED.", output)
        self.assertEqual(
            [(args, kwargs) for args, kwargs in client.sync_send_angles.call_args_list],
            [
                ((list(READY_ANGLES), POSE_SPEED), {"timeout": MOVE_TIMEOUT}),
                ((list(STANDBY_ANGLES), POSE_SPEED), {"timeout": MOVE_TIMEOUT}),
            ],
        )

    def test_wrong_start_still_blocks_transition(self):
        client = client_at_standby()
        client.get_angles.return_value = [20] * 6
        self.assertIn("required starting pose", self.run_menu(client, ["R", "Q"]))
        client.sync_send_angles.assert_not_called()

    def test_individual_joint_still_requires_move_confirmation(self):
        client = client_at_standby()
        self.run_menu(client, ["1", "25", "cancel", "1", "35", "MOVE", "Q"])
        client.send_angle.assert_called_once_with(1, 35, 10)

    def test_manual_joint_speed_persists_and_invalid_entries_keep_it(self):
        client = client_at_standby()
        output = self.run_menu(client, [
            "V", "25", "1", "30", "MOVE",
            "V", "0", "V", "101", "V", "2.5",
            "2", "40", "MOVE", "Q",
        ])
        self.assertIn("Joint speed set to 25.", output)
        self.assertEqual(output.count("Joint speed setting: 25"), 2)
        self.assertEqual(output.count("Speed must be an integer from 1 to 100"), 2)
        self.assertIn("Enter a whole number for joint speed", output)
        self.assertEqual([call.args for call in client.send_angle.call_args_list],
                         [(1, 30, 25), (2, 40, 25)])

    def test_j2_shows_local_and_controller_ranges_and_enforces_local_cap(self):
        client = client_at_standby()
        client.get_joint_min_angle.return_value = -140
        client.get_joint_max_angle.return_value = 140
        output = self.run_menu(client, ["2", "136", "2", "-135", "MOVE", "Q"])
        self.assertIn("=== BUCK-E WORKSPACE CONTROLLER ===", output)
        self.assertIn("1-6 : Move individual joint", output)
        self.assertIn("--- JOINT 2 ---", output)
        self.assertIn("Buck-E range: -135 to 135 degrees", output)
        self.assertIn("Robot configured range: -140.0 to 140.0 degrees", output)
        self.assertIn("Allowed target range: -135.0 to 135.0 degrees", output)
        client.send_angle.assert_called_once_with(2, -135, 10)

    def test_joint_limit_is_rechecked_after_confirmation(self):
        client = client_at_standby()
        client.get_joint_max_angle.side_effect = [168, 20]
        output = self.run_menu(client, ["1", "25", "MOVE", "Q"])
        self.assertIn("outside controller limits", output)
        client.send_angle.assert_not_called()

    def test_fake_mode_runs_same_menu_without_opening_hardware(self):
        arm = FakeArm()
        with patch.object(control, "FakeArm", return_value=arm), \
                patch.object(control, "RealArm") as real, \
                patch("builtins.input", side_effect=["R", "O", "C", "1", "-80", "MOVE", "Q"]), \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(control.main(["--fake"]), 0)
        real.assert_not_called()
        self.assertEqual(arm.get_angles(), (-80,) + READY_ANGLES[1:])
        self.assertFalse(arm.gripper_open)

    def test_interrupt_stops_and_closes(self):
        client = client_at_standby()
        self.run_menu(client, [KeyboardInterrupt()])
        client.stop.assert_called_once_with()
        client.gripper_stop.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
