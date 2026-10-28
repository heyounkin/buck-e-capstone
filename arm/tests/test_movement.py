"""Regression tests for reusable saved-pose movement; no physical hardware."""

import unittest
from unittest.mock import Mock, patch

from bucke_arm.arm import FakeArm, RealArm, READY_ANGLES, STANDBY_ANGLES, POSE_SPEED, MOVE_TIMEOUT


def client_at(pose):
    client = Mock()
    client.get_angles.return_value = list(pose)
    client.is_moving.return_value = 0
    client.get_error_information.return_value = 0
    client.get_joint_min_angle.return_value = -180
    client.get_joint_max_angle.return_value = 180
    return client


class MovementTests(unittest.TestCase):
    def test_both_saved_transitions_send_sdk_target_and_verify_final_pose(self):
        for method, start, target in (("move_to_ready", STANDBY_ANGLES, READY_ANGLES),
                                      ("move_to_standby", READY_ANGLES, STANDBY_ANGLES)):
            with self.subTest(method=method):
                client = client_at(start)
                client.get_angles.side_effect = [start, start, target, target, target]
                with patch("bucke_arm.arm.time.sleep"):
                    self.assertTrue(getattr(RealArm(client), method)())
                client.sync_send_angles.assert_called_once_with(list(target), POSE_SPEED, timeout=MOVE_TIMEOUT)
                client.stop.assert_not_called()

    def test_already_at_target_does_not_send(self):
        for method, target in (("move_to_ready", READY_ANGLES), ("move_to_standby", STANDBY_ANGLES)):
            client = client_at(target)
            self.assertTrue(getattr(RealArm(client), method)())
            client.sync_send_angles.assert_not_called()

    def test_bad_start_status_or_limits_prevent_transmission(self):
        for method, value in (("get_angles", [20] * 6), ("get_angles", None),
                              ("get_angles", [float("nan")] * 6),
                              ("is_moving", 1), ("is_moving", False),
                              ("get_error_information", 4), ("get_error_information", False),
                              ("get_joint_max_angle", 45)):
            with self.subTest(method=method, value=value):
                client = client_at(STANDBY_ANGLES)
                getattr(client, method).return_value = value
                with self.assertRaises((ValueError, RuntimeError)):
                    RealArm(client).move_to_ready()
                client.sync_send_angles.assert_not_called()

    def test_start_pose_is_rechecked_after_limit_queries(self):
        client = client_at(STANDBY_ANGLES)
        client.get_angles.side_effect = [STANDBY_ANGLES, [20] * 6]
        with self.assertRaisesRegex(RuntimeError, "changed"):
            RealArm(client).move_to_ready()
        client.sync_send_angles.assert_not_called()

    def test_sdk_success_return_cannot_override_timeout(self):
        client = client_at(STANDBY_ANGLES)
        client.sync_send_angles.return_value = 1
        with patch("bucke_arm.arm.time.monotonic", side_effect=[0, 5]):
            with self.assertRaisesRegex(RuntimeError, "not confirmed"):
                RealArm(client).move_to_ready()
        client.sync_send_angles.assert_called_once()
        client.stop.assert_called_once()

    def test_post_command_fault_lost_telemetry_and_interrupt_request_stop(self):
        for failure in ("fault", "angles", "interrupt", "write"):
            with self.subTest(failure=failure):
                client = client_at(STANDBY_ANGLES)
                if failure == "fault":
                    client.get_error_information.side_effect = [0, 0, 4]
                elif failure == "angles":
                    client.get_angles.side_effect = [STANDBY_ANGLES, STANDBY_ANGLES, None]
                else:
                    client.sync_send_angles.side_effect = KeyboardInterrupt() if failure == "interrupt" else OSError("write")
                with self.assertRaises((RuntimeError, ValueError, KeyboardInterrupt, OSError)):
                    RealArm(client).move_to_ready()
                client.stop.assert_called_once()
                client.gripper_stop.assert_called_once()

    def test_settling_requires_three_consecutive_target_readings(self):
        client = client_at(STANDBY_ANGLES)
        client.get_angles.side_effect = [STANDBY_ANGLES] * 2 + [READY_ANGLES, STANDBY_ANGLES] + [READY_ANGLES] * 3
        with patch("bucke_arm.arm.time.sleep"):
            self.assertTrue(RealArm(client).move_to_ready())
        self.assertEqual(client.get_angles.call_count, 7)

    def test_manual_joint_and_gripper_use_same_real_arm(self):
        client = client_at(STANDBY_ANGLES)
        arm = RealArm(client)
        arm.move_joint(1, 25)
        arm.open_gripper()
        arm.close_gripper()
        client.send_angle.assert_called_once_with(1, 25, 10)
        self.assertEqual([call.args for call in client.set_gripper_state.call_args_list], [(0, 20), (1, 20)])

    def test_fake_tracks_round_trip_and_manual_changes_without_sdk(self):
        with patch("bucke_arm.arm.metadata.version") as version:
            arm = FakeArm()
            self.assertTrue(arm.move_to_ready())
            self.assertEqual(arm.read_status().joint_angles_deg, READY_ANGLES)
            self.assertTrue(arm.move_to_standby())
            self.assertEqual(arm.get_angles(), STANDBY_ANGLES)
            arm.move_joint(1, 25)
            self.assertEqual(arm.get_angles()[0], 25)
            arm.open_gripper()
            self.assertTrue(arm.gripper_open)
            arm.close_gripper()
            self.assertFalse(arm.gripper_open)
        version.assert_not_called()

    def test_fake_and_real_reject_wrong_saved_start(self):
        for arm in (FakeArm([20] * 6), RealArm(client_at([20] * 6))):
            with self.assertRaises(RuntimeError):
                arm.move_to_ready()

    def test_both_classes_reject_invalid_joint_input(self):
        for arm in (FakeArm(), RealArm(client_at(STANDBY_ANGLES))):
            for joint, target in ((0, 0), (True, 0), (1, float("nan")), (1, 200)):
                with self.assertRaises(ValueError):
                    arm.move_joint(joint, target)


if __name__ == "__main__":
    unittest.main()
