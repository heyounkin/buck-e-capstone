"""Combined sequence command order and fail-closed phase checks; no hardware."""

import contextlib
import io
import itertools
import unittest
from unittest.mock import Mock,patch
from bucke_arm.arm import RealArm,FakeArm,READY_ANGLES
from bucke_arm.grasp import VERIFIED_RAISED_ANGLES,descent_plan
from bucke_arm.pick_and_place import main


class PickAndPlaceTests(unittest.TestCase):
    def client(self):
        client=Mock()
        client.get_angles.return_value=list(VERIFIED_RAISED_ANGLES)
        client.is_moving.return_value=0
        client.get_error_information.return_value=0
        client.get_joint_min_angle.return_value=-180
        client.get_joint_max_angle.return_value=180
        def send_all(command,speed):
            client.get_angles.return_value=list(descent_plan(VERIFIED_RAISED_ANGLES)[0])
            return 1
        def send_one(joint,command,speed):
            actual=list(client.get_angles.return_value)
            actual[joint-1]=-11.6 if joint==2 else 90.08
            client.get_angles.return_value=actual
            return 1
        client.send_angles.side_effect=send_all
        client.send_angle.side_effect=send_one
        return client

    def test_full_phase_order_one_descent_lift_and_sweep(self):
        client=self.client()
        arm=RealArm(client)
        with patch('bucke_arm.arm.time.sleep'),patch('bucke_arm.hover_trajectory.save_forward_trace'),patch.object(arm,'_wait_for_gripper') as wait:
            with contextlib.redirect_stdout(io.StringIO()):
                result=arm.pick_and_place()
        self.assertEqual(client.send_angles.call_count,1)
        self.assertEqual([call.args for call in client.send_angle.call_args_list],[(2,-10.0,5),(1,91.0,5)])
        self.assertEqual([call.args for call in client.set_gripper_state.call_args_list],[(0,20),(1,20),(0,20)])
        commands=[call[0] for call in client.mock_calls if call[0] in ('set_gripper_state','send_angles','send_angle')]
        self.assertEqual(commands,['set_gripper_state','send_angles','set_gripper_state','send_angle','send_angle','set_gripper_state'])
        self.assertEqual(wait.call_count,3)
        self.assertEqual(result['lifted'][1],-11.6)
        self.assertEqual(result['drop'][0],90.08)
        client.sync_send_angles.assert_not_called()

    def test_inadequate_lift_never_sweeps_or_releases(self):
        client=self.client()
        def short_lift(joint,command,speed):
            actual=list(client.get_angles.return_value)
            actual[1]=-13.0  # Inside loose joint tolerance, below clearance gate.
            client.get_angles.return_value=actual
            return 1
        client.send_angle.side_effect=short_lift
        arm=RealArm(client)
        with patch('bucke_arm.arm.time.sleep'),patch('bucke_arm.hover_trajectory.save_forward_trace'),patch.object(arm,'_wait_for_gripper'):
            with contextlib.redirect_stdout(io.StringIO()),self.assertRaisesRegex(RuntimeError,'Lift clearance'):
                arm.pick_and_place()
        self.assertEqual([call.args[0] for call in client.send_angle.call_args_list],[2])
        self.assertEqual([call.args[0] for call in client.set_gripper_state.call_args_list],[0,1])
        self.assertTrue(client.stop.called)

    def test_drop_pose_or_ready_refuses_before_opening(self):
        for pose in (READY_ANGLES,(90.08,-11.6,-110.39,37.17,-1.14,160.04)):
            client=self.client()
            client.get_angles.return_value=list(pose)
            with self.subTest(pose=pose),patch('bucke_arm.arm.time.sleep'),self.assertRaises(RuntimeError):
                RealArm(client).pick_and_place()
            client.set_gripper_state.assert_not_called()
            client.send_angles.assert_not_called()

    def test_future_sweep_limit_refuses_before_any_command(self):
        client=self.client()
        client.get_joint_max_angle.side_effect=lambda joint: 80 if joint==1 else 180
        with patch('bucke_arm.arm.time.sleep'),self.assertRaises(ValueError):
            RealArm(client).pick_and_place()
        client.set_gripper_state.assert_not_called()
        client.send_angles.assert_not_called()

    def test_gripper_error_stops_before_descent(self):
        client=self.client()
        client.is_gripper_moving.return_value=-1
        with patch('bucke_arm.arm.time.sleep'),self.assertRaisesRegex(RuntimeError,'Invalid gripper'):
            RealArm(client).pick_and_place()
        client.send_angles.assert_not_called()
        client.send_angle.assert_not_called()
        client.stop.assert_called_once()

    def test_gripper_wait_does_not_accept_instant_stale_stopped_reply(self):
        client=self.client()
        client.is_gripper_moving.return_value=0
        with patch('bucke_arm.arm.time.sleep'),patch('bucke_arm.arm.time.monotonic',side_effect=itertools.count(step=.2)):
            RealArm(client)._wait_for_gripper()
        self.assertGreater(client.is_gripper_moving.call_count,3)

    def test_gripper_timeout_has_no_fallback_motion(self):
        client=self.client()
        client.is_gripper_moving.return_value=1
        with patch('bucke_arm.arm.time.sleep'),patch('bucke_arm.arm.time.monotonic',side_effect=itertools.count(step=.5)):
            with self.assertRaisesRegex(RuntimeError,'did not settle'):
                RealArm(client)._wait_for_gripper()
        client.send_angles.assert_not_called()
        client.send_angle.assert_not_called()

    def test_loaded_closure_accepts_stable_partial_position_with_moving_flag(self):
        client=self.client()
        client.is_gripper_moving.return_value=1
        client.get_gripper_value.return_value=54
        with patch('bucke_arm.arm.time.sleep'),patch('bucke_arm.arm.time.monotonic',side_effect=itertools.count(step=.1)):
            with contextlib.redirect_stdout(io.StringIO()):
                RealArm(client)._wait_for_gripper(allow_contact=True)
        self.assertGreater(client.get_gripper_value.call_count,3)
        client.set_gripper_state.assert_not_called()
        client.send_angle.assert_not_called()

    def test_loaded_closure_rejects_open_invalid_or_changing_values(self):
        for label,value in [('open',100),('invalid',-1),('changing',None)]:
            client=self.client()
            client.is_gripper_moving.return_value=1
            if value is None:
                client.get_gripper_value.side_effect=[20,30,40,50]*20
            else:
                client.get_gripper_value.return_value=value
            with self.subTest(case=label),patch('bucke_arm.arm.time.sleep'),patch('bucke_arm.arm.time.monotonic',side_effect=itertools.count(step=.2)):
                with self.assertRaises(RuntimeError):
                    RealArm(client)._wait_for_gripper(allow_contact=True)
            client.send_angle.assert_not_called()
            client.set_gripper_state.assert_not_called()

    def test_resume_observed_held_pose_never_opens_before_lift(self):
        client=self.client()
        held=(33.92,-16.08,-110.12,36.56,-.96,160.57)
        client.get_angles.return_value=list(held)
        arm=RealArm(client)
        with patch('bucke_arm.arm.time.sleep'),patch('bucke_arm.hover_trajectory.save_forward_trace'),patch.object(arm,'_wait_for_gripper') as wait:
            with contextlib.redirect_stdout(io.StringIO()):
                result=arm.finish_held_pickup()
        self.assertEqual(result['grasp'],held)
        self.assertEqual([call.args for call in client.send_angle.call_args_list],[(2,-10.0,5),(1,91.0,5)])
        client.send_angles.assert_not_called()
        client.set_gripper_state.assert_called_once_with(0,20)
        actions=[call[0] for call in client.mock_calls if call[0] in ('send_angle','set_gripper_state')]
        self.assertEqual(actions,['send_angle','send_angle','set_gripper_state'])
        self.assertEqual(wait.call_args_list[0].kwargs,{'allow_contact':True})

    def test_resume_from_ready_or_drop_never_moves_or_opens(self):
        for pose in (READY_ANGLES,(90.08,-11.6,-110.39,37.17,-1.14,160.04)):
            client=self.client()
            client.get_angles.return_value=list(pose)
            with patch('bucke_arm.arm.time.sleep'),self.assertRaises(RuntimeError):
                RealArm(client).finish_held_pickup()
            client.send_angle.assert_not_called()
            client.set_gripper_state.assert_not_called()

    def test_preview_and_fake_never_connect_to_real_arm(self):
        with patch('bucke_arm.pick_and_place.RealArm') as constructor:
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main([]),0)
                self.assertEqual(main(['--fake']),0)
                self.assertEqual(main(['--fake','--return-ready']),0)
                self.assertEqual(main(['--fake','--resume-grasp','--return-ready']),0)
        constructor.assert_not_called()
        arm=FakeArm(VERIFIED_RAISED_ANGLES)
        result=arm.pick_and_place()
        self.assertTrue(arm.gripper_open)
        self.assertEqual(result['drop'][0],90.08)
        with self.assertRaises(RuntimeError):
            arm.pick_and_place()


if __name__=='__main__':
    unittest.main()
