"""Checks for measured-reference, one-command raised positioning trial."""

import contextlib
import io
import unittest
from unittest.mock import Mock,patch
from bucke_arm.arm import RealArm,FakeArm,READY_ANGLES
from bucke_arm.local_approach import START_ANGLES,SETTLED_OBSERVATION,desired_target,command_target,main
from bucke_arm.kinematics import gripper_midpoint_mat
from bucke_arm.hover_alignment import tilt_degrees


class LocalApproachTests(unittest.TestCase):
    def client(self):
        client=Mock()
        client.get_angles.return_value=list(START_ANGLES)
        client.is_moving.return_value=0
        client.get_error_information.return_value=0
        client.get_joint_min_angle.return_value=-180
        client.get_joint_max_angle.return_value=180
        def send(command,speed):
            client.get_angles.return_value=list(desired_target())
            return 1
        client.send_angles.side_effect=send
        return client

    def test_relative_target_uses_measured_delta_not_absolute_fk(self):
        before=gripper_midpoint_mat(START_ANGLES)
        after=gripper_midpoint_mat(desired_target())
        for a,b,d in zip(before,after,(.375,.25,-.375)):
            self.assertAlmostEqual((b-a)/25.4,d,delta=.001)
        self.assertLess(tilt_degrees(desired_target()),.01)
        self.assertEqual(command_target()[5],START_ANGLES[5])
        self.assertLessEqual(max(abs(a-b) for a,b in zip(command_target(),desired_target())),2)

    def test_completion_compares_desired_angles_not_precompensated_command(self):
        client=self.client()
        with patch('bucke_arm.arm.time.sleep'),patch('bucke_arm.hover_trajectory.save_forward_trace') as save:
            with contextlib.redirect_stdout(io.StringIO()):
                actual=RealArm(client).move_to_local_approach()
        self.assertEqual(actual['actual'],desired_target())
        client.send_angles.assert_called_once_with(list(command_target()),5)
        client.send_angle.assert_not_called()
        client.set_gripper_state.assert_not_called()
        self.assertTrue(save.call_args.args[1])
        self.assertEqual(save.call_args.kwargs['command_angles'],command_target())
        self.assertEqual(save.call_args.kwargs['arrival_angles'],desired_target())

    def test_observed_settling_is_accepted_and_live_j6_is_held(self):
        client=self.client()
        client.get_angles.return_value=list(SETTLED_OBSERVATION)
        desired=desired_target(SETTLED_OBSERVATION)
        def send(command,speed):
            client.get_angles.return_value=list(desired)
            return 1
        client.send_angles.side_effect=send
        with patch('bucke_arm.arm.time.sleep'),patch('bucke_arm.hover_trajectory.save_forward_trace'):
            with contextlib.redirect_stdout(io.StringIO()):
                result=RealArm(client).move_to_local_approach()
        self.assertEqual(result['start'],SETTLED_OBSERVATION)
        self.assertEqual(result['command'][5],159.96)
        self.assertEqual(result['actual'],desired)
        client.send_angles.assert_called_once_with(list(command_target(SETTLED_OBSERVATION)),5)
        for a,b in zip(gripper_midpoint_mat(desired),gripper_midpoint_mat(desired_target())):
            self.assertAlmostEqual(a,b,delta=.01)

    def test_unstable_local_samples_never_move(self):
        client=self.client()
        client.get_angles.side_effect=[START_ANGLES,SETTLED_OBSERVATION]
        with patch('bucke_arm.arm.time.sleep'),self.assertRaisesRegex(RuntimeError,'not stable'):
            RealArm(client).move_to_local_approach()
        client.send_angles.assert_not_called()

    def test_far_pose_and_post_limit_change_never_move(self):
        client=self.client()
        shifted=list(START_ANGLES)
        shifted[1]+=2
        client.get_angles.return_value=shifted
        with patch('bucke_arm.arm.time.sleep'),self.assertRaises(RuntimeError):
            RealArm(client).move_to_local_approach()
        client.send_angles.assert_not_called()
        client=self.client()
        client.get_angles.side_effect=[START_ANGLES]*3+[SETTLED_OBSERVATION]*3
        with patch('bucke_arm.arm.time.sleep'),self.assertRaisesRegex(RuntimeError,'changed during'):
            RealArm(client).move_to_local_approach()
        client.send_angles.assert_not_called()

    def test_wrong_start_never_moves(self):
        client=self.client()
        client.get_angles.return_value=list(READY_ANGLES)
        with patch('bucke_arm.arm.time.sleep'),self.assertRaises(RuntimeError):
            RealArm(client).move_to_local_approach()
        client.send_angles.assert_not_called()

    def test_command_limits_reject_before_motion(self):
        client=self.client()
        def high(joint):
            return 33 if joint==4 else 180
        client.get_joint_max_angle.side_effect=high
        with patch('bucke_arm.arm.time.sleep'),self.assertRaises(ValueError):
            RealArm(client).move_to_local_approach()
        client.send_angles.assert_not_called()

    def test_failure_stops_and_records_without_closure(self):
        client=self.client()
        def send(*args):
            client.get_angles.return_value=None
            return 1
        client.send_angles.side_effect=send
        with patch('bucke_arm.arm.time.sleep'),patch('bucke_arm.hover_trajectory.save_forward_trace') as save:
            with contextlib.redirect_stdout(io.StringIO()),self.assertRaises(ValueError):
                RealArm(client).move_to_local_approach()
        client.stop.assert_called_once()
        client.set_gripper_state.assert_not_called()
        self.assertFalse(save.call_args.args[1])
        self.assertEqual(client.send_angles.call_count,1)

    def test_preview_never_connects_and_fake_preserves_jaws(self):
        with patch('bucke_arm.local_approach.RealArm') as constructor:
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main([]),0)
        constructor.assert_not_called()
        arm=FakeArm(START_ANGLES)
        arm.open_gripper()
        self.assertEqual(arm.move_to_local_approach()['actual'],desired_target())
        self.assertTrue(arm.gripper_open)


if __name__=='__main__':
    unittest.main()
