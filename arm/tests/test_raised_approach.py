"""Raised J2-only test: no wrist/gripper commands or route joins."""

import contextlib
import io
import unittest
from unittest.mock import Mock, patch

from bucke_arm.arm import READY_ANGLES, RealArm, FakeArm
from bucke_arm.kinematics import gripper_midpoint_mat
from bucke_arm.raised_approach import START_ANGLES, raised_target, main


class RaisedApproachTests(unittest.TestCase):
    def client(self):
        client = Mock()
        client.get_angles.return_value = list(START_ANGLES)
        client.is_moving.return_value = 0
        client.get_error_information.return_value = 0
        client.get_joint_min_angle.return_value = -180
        client.get_joint_max_angle.return_value = 180
        def send(joint, angle, speed):
            q = list(client.get_angles.return_value)
            q[joint - 1] = angle
            client.get_angles.return_value = q
            return 1
        client.send_angle.side_effect = send
        return client

    def test_only_j2_changes_and_nominal_height_increases_throughout(self):
        target = raised_target()
        self.assertEqual(target[:1] + target[2:], START_ANGLES[:1] + START_ANGLES[2:])
        self.assertGreater(target[1], START_ANGLES[1])
        self.assertAlmostEqual(gripper_midpoint_mat(target)[2] / 25.4, 1.625, places=4)
        heights = []
        for i in range(101):
            q = tuple(a+(b-a)*i/100 for a,b in zip(START_ANGLES,target))
            heights.append(gripper_midpoint_mat(q)[2])
        self.assertTrue(all(b >= a for a,b in zip(heights,heights[1:])))

    def test_real_sends_one_j2_command_and_records_new_direction(self):
        client = self.client()
        with patch('bucke_arm.arm.time.sleep'), patch('bucke_arm.hover_trajectory.save_forward_trace') as save:
            with contextlib.redirect_stdout(io.StringIO()):
                actual = RealArm(client).move_to_raised_approach()
        self.assertEqual(actual, raised_target())
        client.send_angle.assert_called_once_with(2, raised_target()[1], 5)
        client.send_angles.assert_not_called()
        client.sync_send_angles.assert_not_called()
        client.set_gripper_state.assert_not_called()
        self.assertEqual(save.call_args.kwargs['direction'], 'stopped_to_raised_approach')
        self.assertTrue(save.call_args.args[1])

    def test_ready_and_changed_poses_never_move(self):
        changed = list(START_ANGLES)
        changed[5] += 1
        for pose in (READY_ANGLES, changed):
            client = self.client()
            client.get_angles.return_value = list(pose)
            with self.subTest(pose=pose), patch('bucke_arm.arm.time.sleep'):
                with self.assertRaisesRegex(RuntimeError, 'recorded J2-test pose'):
                    RealArm(client).move_to_raised_approach()
            client.send_angle.assert_not_called()
            client.send_angles.assert_not_called()

    def test_limits_refuse_before_motion(self):
        client = self.client()
        client.get_joint_max_angle.return_value = -15
        with patch('bucke_arm.arm.time.sleep'), self.assertRaises(ValueError):
            RealArm(client).move_to_raised_approach()
        client.send_angle.assert_not_called()

    def test_bad_read_requests_stop_without_retry_or_closure(self):
        client = self.client()
        def send(*args):
            client.get_angles.return_value = None
            return 1
        client.send_angle.side_effect = send
        with patch('bucke_arm.arm.time.sleep'), patch('bucke_arm.hover_trajectory.save_forward_trace') as save:
            with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(ValueError):
                RealArm(client).move_to_raised_approach()
        self.assertEqual(client.send_angle.call_count, 1)
        client.stop.assert_called_once()
        client.set_gripper_state.assert_not_called()
        self.assertFalse(save.call_args.args[1])

    def test_fake_preserves_jaws_and_rejects_ready(self):
        arm = FakeArm(START_ANGLES)
        arm.open_gripper()
        self.assertEqual(arm.move_to_raised_approach(), raised_target())
        self.assertTrue(arm.gripper_open)
        with self.assertRaises(RuntimeError):
            FakeArm(READY_ANGLES).move_to_raised_approach()

    def test_preview_never_connects(self):
        with patch('bucke_arm.raised_approach.RealArm') as constructor:
            with contextlib.redirect_stdout(io.StringIO()) as output:
                self.assertEqual(main([]), 0)
        constructor.assert_not_called()
        self.assertIn('J2-only', output.getvalue())


if __name__ == '__main__':
    unittest.main()
