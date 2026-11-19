"""Check candidate pickup geometry without connecting to hardware."""

import contextlib
import io
import unittest
from unittest.mock import Mock, patch

from bucke_arm.arm import SPEC_LIMITS, READY_ANGLES, RealArm, FakeArm
from bucke_arm.kinematics import forward_kinematics, gripper_midpoint_mat
from bucke_arm.pickup import main, pickup_target, require_grasp_position
from bucke_arm.resume_hover import HOVER


class PickupTests(unittest.TestCase):
    def setUp(self):
        # Retain coverage of the underlying sequence with the hardware gate
        # mocked only in tests. The separate lockout tests exercise production.
        gate = patch('bucke_arm.pickup.require_pickup_execution')
        gate.start()
        self.addCleanup(gate.stop)

    def test_candidates_reach_block_and_retain_downward_tilt_inside_limits(self):
        for height in (0.6, 0.625, 0.75):
            with self.subTest(height=height):
                target = pickup_target(height)
                xyz = gripper_midpoint_mat(target)
                for actual, expected in zip(xyz, (177.8, 50.8, height * 25.4)):
                    self.assertAlmostEqual(actual, expected, delta=0.02)
                self.assertTrue(all(low <= value <= high for value, (low, high) in zip(target, SPEC_LIMITS)))
                self.assertAlmostEqual(sum(target[1:4]), sum(HOVER[1:4]), places=3)
                self.assertEqual(target[4:], HOVER[4:])
                self.assertLess(forward_kinematics(target).rotation_matrix[2][2], -0.999)

    def test_default_leaves_nominal_eighth_inch_below_fingers(self):
        xyz = gripper_midpoint_mat(pickup_target())
        self.assertAlmostEqual(xyz[2] / 25.4 - 0.5, 0.125, delta=0.001)

    def test_invalid_grasp_heights_are_refused(self):
        for height in (0, 0.25, 0.5, 1, -0.1, 1.1, float('nan'), float('inf'), True, '0.5'):
            with self.subTest(height=height), self.assertRaises(ValueError):
                pickup_target(height)

    def test_preview_cannot_connect_to_hardware(self):
        with patch('bucke_arm.pickup.RealArm') as constructor:
            with contextlib.redirect_stdout(io.StringIO()) as output:
                self.assertEqual(main([]), 0)
            constructor.assert_not_called()
        self.assertIn('Preview only', output.getvalue())

    def client(self):
        client = Mock()
        client.get_angles.return_value = list(READY_ANGLES)
        client.is_moving.return_value = 0
        client.get_error_information.return_value = 0
        client.get_joint_min_angle.return_value = -180
        client.get_joint_max_angle.return_value = 180
        def send(angles, speed):
            client.get_angles.return_value = list(angles)
            return 1
        client.send_angles.side_effect = send
        return client

    def test_one_approach_then_close_with_pickup_trace(self):
        client = self.client()
        def close(state, speed):
            self.assertEqual(state, 1)
            self.assertEqual(client.send_angles.call_count, 1)
            self.assertEqual(client.get_angles.return_value, list(pickup_target()))
        client.set_gripper_state.side_effect = close
        with patch('bucke_arm.arm.time.sleep'), patch('bucke_arm.hover_trajectory.save_forward_trace') as save:
            with contextlib.redirect_stdout(io.StringIO()):
                actual = RealArm(client).pick_up_block()
        self.assertEqual(actual, pickup_target())
        client.send_angles.assert_called_once_with(list(pickup_target()), 5)
        client.set_gripper_state.assert_called_once_with(1, 20)
        client.sync_send_angles.assert_not_called()
        self.assertEqual(save.call_args.kwargs['direction'], 'ready_to_pickup')
        self.assertTrue(save.call_args.args[1])

    def test_invalid_start_or_controller_limit_never_moves_or_closes(self):
        for bad_start in (True, False):
            client = self.client()
            if bad_start:
                client.get_angles.return_value = [0] * 6
            else:
                client.get_joint_max_angle.return_value = 10
            with self.subTest(bad_start=bad_start), patch('bucke_arm.arm.time.sleep'):
                with self.assertRaises((RuntimeError, ValueError)):
                    RealArm(client).pick_up_block()
            client.send_angles.assert_not_called()
            client.set_gripper_state.assert_not_called()

    def test_motion_read_failure_stops_without_closing(self):
        client = self.client()
        def send(*args):
            client.get_angles.return_value = None
            return 1
        client.send_angles.side_effect = send
        with patch('bucke_arm.arm.time.sleep'), patch('bucke_arm.hover_trajectory.save_forward_trace') as save:
            with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(ValueError):
                RealArm(client).pick_up_block()
        client.set_gripper_state.assert_not_called()
        self.assertTrue(client.stop.called)
        self.assertFalse(save.call_args.args[1])

    def test_fk_endpoint_failure_prevents_closure(self):
        client = self.client()
        def reached(target, tolerance, **kwargs):
            client.get_angles.return_value = list(target)
        arm = RealArm(client)
        with patch('bucke_arm.arm.time.sleep'), patch.object(arm, '_record_forward_hover', side_effect=reached):
            with patch('bucke_arm.pickup.require_grasp_position', side_effect=RuntimeError('missed block')):
                with self.assertRaisesRegex(RuntimeError, 'missed block'):
                    arm.pick_up_block()
        client.set_gripper_state.assert_not_called()
        client.stop.assert_called_once()

    def test_endpoint_bounds_reject_table_level_and_wrong_xy(self):
        target = pickup_target()
        for xyz in ((7, 2, 0.5), (7.2, 2, 0.625), (7, 2, 1)):
            with self.subTest(xyz=xyz), patch('bucke_arm.pickup.gripper_midpoint_mat', return_value=tuple(v*25.4 for v in xyz)):
                with self.assertRaises(RuntimeError):
                    require_grasp_position(target)

    def test_fake_matches_ready_only_approach_and_closure(self):
        arm = FakeArm(READY_ANGLES)
        arm.open_gripper()
        self.assertEqual(arm.pick_up_block(), pickup_target())
        self.assertFalse(arm.gripper_open)
        with self.assertRaises(RuntimeError):
            arm.pick_up_block()

    def test_cancel_closes_port_without_approach(self):
        client = self.client()
        with patch('bucke_arm.pickup.RealArm', return_value=RealArm(client)):
            with patch('builtins.input', return_value=''), patch('bucke_arm.arm.time.sleep'):
                with contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(main(['--execute']), 0)
        client.send_angles.assert_not_called()
        client.set_gripper_state.assert_not_called()
        client._serial_port.close.assert_called_once()

    def test_execute_uses_pickup_and_reports_closure(self):
        arm = Mock()
        arm.pick_up_block.return_value = pickup_target()
        with patch('bucke_arm.pickup.RealArm', return_value=arm), patch('builtins.input', return_value='PICKUP'):
            with contextlib.redirect_stdout(io.StringIO()) as output:
                self.assertEqual(main(['--execute']), 0)
        arm.pick_up_block.assert_called_once_with(0.625)
        arm.close.assert_called_once()
        self.assertIn('jaw closure commanded', output.getvalue())


class PickupLockoutTests(unittest.TestCase):
    def test_cli_refuses_before_connection_or_confirmation(self):
        with patch('bucke_arm.pickup.RealArm') as constructor, patch('builtins.input') as prompt:
            with contextlib.redirect_stdout(io.StringIO()) as output:
                self.assertEqual(main(['--execute']), 1)
        constructor.assert_not_called()
        prompt.assert_not_called()
        self.assertIn('disabled', output.getvalue())

    def test_direct_real_arm_refuses_before_queries_or_commands(self):
        client = Mock()
        with self.assertRaisesRegex(RuntimeError, 'disabled'):
            RealArm(client).pick_up_block()
        self.assertEqual(client.mock_calls, [])


if __name__ == '__main__':
    unittest.main()
