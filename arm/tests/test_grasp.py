"""No hardware: one relative descent and guarded closure from verified pose."""

import contextlib
import io
import unittest
from unittest.mock import Mock,patch
from bucke_arm.arm import RealArm,FakeArm,READY_ANGLES
from bucke_arm.grasp import VERIFIED_RAISED_ANGLES,known_grasp_start,descent_plan,require_descent_result,main
from bucke_arm.kinematics import gripper_midpoint_mat


class GraspTests(unittest.TestCase):
    def client(self):
        client=Mock()
        client.get_angles.return_value=list(VERIFIED_RAISED_ANGLES)
        client.is_moving.return_value=0
        client.get_error_information.return_value=0
        client.get_joint_min_angle.return_value=-180
        client.get_joint_max_angle.return_value=180
        def send(command,speed):
            client.get_angles.return_value=list(descent_plan(VERIFIED_RAISED_ANGLES)[0])
            return 1
        client.send_angles.side_effect=send
        return client

    def test_positive_near_zero_j5_is_allowed_with_geometry_checks(self):
        live=(34.18,-14.06,-108.36,32.34,.70,160.40)
        self.assertTrue(known_grasp_start(live))
        for j5 in (-3,3):
            bad=list(live)
            bad[4]=j5
            self.assertFalse(known_grasp_start(bad))
        bad=list(live)
        bad[1]+=2
        self.assertFalse(known_grasp_start(bad))

    def test_latest_subdegree_arrival_is_accepted_with_same_position_and_tilt_gates(self):
        live=(34.1,-13.88,-107.84,32.25,.61,160.48)
        self.assertTrue(known_grasp_start(live))
        from bucke_arm.arm import APPROACH_TOLERANCE
        from bucke_arm.grasp import SETTLE_ALLOWANCE,grasp_start_problem
        self.assertEqual(SETTLE_ALLOWANCE,(APPROACH_TOLERANCE,)*6)
        bad=list(live)
        bad[2]+=2
        self.assertFalse(known_grasp_start(bad))
        self.assertIn('J3',grasp_start_problem(bad))

    def test_relative_descent_preserves_xy_and_existing_bend_and_wrist(self):
        desired,command=descent_plan(VERIFIED_RAISED_ANGLES)
        before=gripper_midpoint_mat(VERIFIED_RAISED_ANGLES)
        after=gripper_midpoint_mat(desired)
        for a,b,delta in zip(before,after,(0,0,-.25)):
            self.assertAlmostEqual((b-a)/25.4,delta,delta=.001)
        self.assertEqual(command[4:],VERIFIED_RAISED_ANGLES[4:])
        self.assertAlmostEqual(sum(desired[1:4]),sum(VERIFIED_RAISED_ANGLES[1:4]),places=3)

    def test_single_descent_before_closure_and_no_lift(self):
        client=self.client()
        desired,command=descent_plan(VERIFIED_RAISED_ANGLES)
        def close(state,speed):
            self.assertEqual(client.send_angles.call_count,1)
            self.assertEqual(tuple(client.get_angles.return_value),desired)
            self.assertEqual(state,1)
        client.set_gripper_state.side_effect=close
        with patch('bucke_arm.arm.time.sleep'),patch('bucke_arm.hover_trajectory.save_forward_trace') as save:
            with contextlib.redirect_stdout(io.StringIO()):
                result=RealArm(client).grasp_block()
        client.send_angles.assert_called_once_with(list(command),5)
        client.set_gripper_state.assert_called_once_with(1,20)
        client.send_angle.assert_not_called()
        self.assertEqual(result['actual'],desired)
        self.assertEqual(save.call_args.kwargs['direction'],'raised_approach_to_grasp')

    def test_start_region_or_limits_refuse_before_movement(self):
        for bad_start in (True,False):
            client=self.client()
            if bad_start:
                client.get_angles.return_value=list(READY_ANGLES)
            else:
                client.get_joint_max_angle.side_effect=lambda joint: 37 if joint==4 else 180
            with self.subTest(bad_start=bad_start),patch('bucke_arm.arm.time.sleep'):
                with self.assertRaises((RuntimeError,ValueError)):
                    RealArm(client).grasp_block()
            client.send_angles.assert_not_called()
            client.set_gripper_state.assert_not_called()

    def test_capture_fault_stops_without_closing(self):
        client=self.client()
        def send(*args):
            client.get_error_information.return_value=1
            return 1
        client.send_angles.side_effect=send
        with patch('bucke_arm.arm.time.sleep'),patch('bucke_arm.hover_trajectory.save_forward_trace'):
            with contextlib.redirect_stdout(io.StringIO()),self.assertRaises(RuntimeError):
                RealArm(client).grasp_block()
        client.stop.assert_called_once()
        client.set_gripper_state.assert_not_called()

    def test_failed_geometry_blocks_closure(self):
        client=self.client()
        with patch('bucke_arm.arm.time.sleep'),patch('bucke_arm.hover_trajectory.save_forward_trace'):
            with patch('bucke_arm.grasp.require_descent_result',side_effect=RuntimeError('relative check failed')):
                with contextlib.redirect_stdout(io.StringIO()),self.assertRaisesRegex(RuntimeError,'relative check failed'):
                    RealArm(client).grasp_block()
        client.stop.assert_called_once()
        client.set_gripper_state.assert_not_called()

    def test_no_descent_and_excessive_descent_are_rejected(self):
        desired,_=descent_plan(VERIFIED_RAISED_ANGLES)
        with self.assertRaises(RuntimeError):
            require_descent_result(VERIFIED_RAISED_ANGLES,VERIFIED_RAISED_ANGLES,VERIFIED_RAISED_ANGLES)
        with patch('bucke_arm.grasp.gripper_midpoint_mat',side_effect=[(0,0,100),(0,0,80)]):
            with self.assertRaises(RuntimeError):
                require_descent_result(VERIFIED_RAISED_ANGLES,desired,desired)

    def test_preview_and_fake_do_not_touch_hardware(self):
        with patch('bucke_arm.grasp.RealArm') as constructor:
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main([]),0)
        constructor.assert_not_called()
        arm=FakeArm(VERIFIED_RAISED_ANGLES)
        arm.open_gripper()
        result=arm.grasp_block()
        self.assertFalse(arm.gripper_open)
        self.assertEqual(result['actual'],descent_plan(VERIFIED_RAISED_ANGLES)[0])


if __name__=='__main__':
    unittest.main()
