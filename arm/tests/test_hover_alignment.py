"""Model and command checks for the local high-hover orientation test."""

import contextlib
import io
import unittest
from unittest.mock import Mock, patch

from bucke_arm.arm import RealArm, FakeArm, READY_ANGLES
from bucke_arm.hover_alignment import START_ANGLES, alignment_target, tilt_degrees, main
from bucke_arm.kinematics import gripper_midpoint_mat


class AlignmentTests(unittest.TestCase):
    def client(self):
        client = Mock()
        client.get_angles.return_value = list(START_ANGLES)
        client.is_moving.return_value = 0
        client.get_error_information.return_value = 0
        client.get_joint_min_angle.return_value = -180
        client.get_joint_max_angle.return_value = 180
        def send(target, speed):
            client.get_angles.return_value = list(target)
            return 1
        client.send_angles.side_effect = send
        return client

    def test_model_squares_flange_preserves_midpoint_and_signed_j6(self):
        target = alignment_target()
        self.assertLess(tilt_degrees(target), 0.01)
        self.assertEqual(target[5], START_ANGLES[5])
        for a,b in zip(gripper_midpoint_mat(target), gripper_midpoint_mat(START_ANGLES)):
            self.assertAlmostEqual(a,b,delta=0.01)

    def test_one_command_without_closure_or_retry(self):
        client = self.client()
        with patch('bucke_arm.arm.time.sleep'), patch('bucke_arm.hover_trajectory.save_forward_trace') as save:
            with contextlib.redirect_stdout(io.StringIO()):
                actual = RealArm(client).square_hover()
        self.assertEqual(actual, alignment_target())
        client.send_angles.assert_called_once_with(list(alignment_target()), 5)
        client.send_angle.assert_not_called()
        client.sync_send_angles.assert_not_called()
        client.set_gripper_state.assert_not_called()
        self.assertEqual(save.call_args.kwargs['direction'], 'high_hover_alignment')

    def test_ready_and_shifted_pose_refuse_before_command(self):
        shifted = list(START_ANGLES)
        shifted[1] += 1
        for pose in (READY_ANGLES, shifted, (33.92, -13.35, -93.95, 12.83, 0.52, 160.48)):
            client = self.client()
            client.get_angles.return_value = list(pose)
            with self.subTest(pose=pose), patch('bucke_arm.arm.time.sleep'):
                with self.assertRaisesRegex(RuntimeError,'recorded high-hover pose'):
                    RealArm(client).square_hover()
            client.send_angles.assert_not_called()

    def test_changed_pose_after_limit_queries_refuses(self):
        client = self.client()
        def limit(joint):
            client.get_angles.return_value = list(READY_ANGLES)
            return 180
        client.get_joint_max_angle.side_effect = limit
        with patch('bucke_arm.arm.time.sleep'), self.assertRaises(RuntimeError):
            RealArm(client).square_hover()
        client.send_angles.assert_not_called()

    def test_fault_during_capture_stops_and_preserves_failure(self):
        client = self.client()
        def send(target,speed):
            client.get_error_information.return_value = 1
            return 1
        client.send_angles.side_effect = send
        with patch('bucke_arm.arm.time.sleep'), patch('bucke_arm.hover_trajectory.save_forward_trace') as save:
            with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(RuntimeError):
                RealArm(client).square_hover()
        self.assertEqual(client.send_angles.call_count,1)
        client.stop.assert_called_once()
        client.set_gripper_state.assert_not_called()
        self.assertFalse(save.call_args.args[1])

    def test_fake_matches_start_and_preserves_jaws(self):
        arm = FakeArm(START_ANGLES)
        arm.open_gripper()
        self.assertEqual(arm.square_hover(),alignment_target())
        self.assertTrue(arm.gripper_open)
        with self.assertRaises(RuntimeError):
            FakeArm(READY_ANGLES).square_hover()

    def test_preview_does_not_connect(self):
        with patch('bucke_arm.hover_alignment.RealArm') as constructor:
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main([]),0)
        constructor.assert_not_called()


if __name__ == '__main__':
    unittest.main()
