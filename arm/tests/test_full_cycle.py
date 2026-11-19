"""READY-to-READY orchestration and stopping boundaries without hardware."""

import contextlib
import io
import unittest
from unittest.mock import Mock,patch
from bucke_arm.arm import RealArm,FakeArm,READY_ANGLES
from bucke_arm.full_cycle import HIGH_HOVER_OBSERVED,POST_HOVER_OBSERVED,preplaced_pickup,confirm_pickup,main
from bucke_arm.grasp import VERIFIED_RAISED_ANGLES,descent_plan


class FullCycleTests(unittest.TestCase):
    def client(self):
        client=Mock()
        client.get_angles.return_value=list(READY_ANGLES)
        client.is_moving.return_value=0
        client.get_error_information.return_value=0
        client.get_joint_min_angle.return_value=-180
        client.get_joint_max_angle.return_value=180
        def all_joints(command,speed):
            n=client.send_angles.call_count
            if command[0]==-90 and command[4]==90:
                client.get_angles.return_value=list(command)
                return 1
            if n==1:
                q=HIGH_HOVER_OBSERVED
            elif n==2:
                q=POST_HOVER_OBSERVED
            elif n==3:
                q=VERIFIED_RAISED_ANGLES
            else:
                q=descent_plan(tuple(client.get_angles.return_value))[0]
            client.get_angles.return_value=list(q)
            return 1
        def one_joint(joint,value,speed):
            q=list(client.get_angles.return_value)
            q[joint-1]=(-11.6 if joint==2 and value==-10 else
                         90.08 if joint==1 and value==91 else value)
            client.get_angles.return_value=q
            return 1
        client.send_angles.side_effect=all_joints
        client.send_angle.side_effect=one_joint
        return client

    def run_arm(self,client,pickup=lambda *args:True,ret=lambda *args:True):
        arm=RealArm(client)
        with patch('bucke_arm.arm.time.sleep'),patch('bucke_arm.hover_trajectory.save_forward_trace'),patch.object(arm,'_wait_for_gripper'):
            with contextlib.redirect_stdout(io.StringIO()):
                return arm.run_ready_cycle(pickup,ret)

    def test_ready_approach_place_pickup_drop_return_order(self):
        client=self.client()
        checkpoints=[]
        def pickup(q):
            checkpoints.append('place')
            self.assertEqual(q,VERIFIED_RAISED_ANGLES)
            self.assertEqual(client.send_angles.call_count,3)
            self.assertEqual([call.args[0] for call in client.set_gripper_state.call_args_list],[0])
            return True
        def ret(index,*args):
            checkpoints.append(index)
            self.assertEqual([call.args[0] for call in client.set_gripper_state.call_args_list],[0,1,0])
            return True
        result=self.run_arm(client,pickup,ret)
        self.assertEqual(result['ready'],READY_ANGLES)
        self.assertEqual(checkpoints,['place',1,2,3,4,5])
        self.assertEqual(client.send_angles.call_count,5)
        self.assertEqual([call.args for call in client.send_angle.call_args_list],
                         [(2,-10.0,5),(1,91.0,5),(2,0.0,5),(6,-135.0,10),
                          (3,0.0,5),(4,0.0,5)])
        client.sync_send_angles.assert_not_called()

    def test_full_cycle_handles_observed_positive_j5_without_late_return_refusal(self):
        client=self.client()
        original=client.send_angles.side_effect
        observed=(
            (34.01,-13.35,-93.95,12.91,.79,160.4),
            (34.18,-9.66,-104.23,20.47,.7,160.4),
            (34.18,-14.06,-108.36,32.34,.7,160.4),
        )
        def all_joints(command,speed):
            n=client.send_angles.call_count
            if n<=3:
                client.get_angles.return_value=list(observed[n-1])
                return 1
            return original(command,speed)
        client.send_angles.side_effect=all_joints
        result=self.run_arm(client)
        self.assertEqual(result['ready'],READY_ANGLES)
        self.assertEqual(result['cycle']['grasp'][4],.7)
        self.assertEqual(result['cycle']['drop'][4],.7)

    def test_latest_reported_arrival_can_complete_handoffs_to_grasp_and_return(self):
        client=self.client()
        original=client.send_angles.side_effect
        observed=(
            (33.92,-13.35,-93.95,12.91,-.08,160.48),
            (34.1,-9.75,-104.15,20.39,.7,160.48),
            (34.1,-13.88,-107.84,32.25,.61,160.48),
        )
        def all_joints(command,speed):
            n=client.send_angles.call_count
            if n<=3:
                client.get_angles.return_value=list(observed[n-1])
                return 1
            return original(command,speed)
        client.send_angles.side_effect=all_joints
        result=self.run_arm(client)
        self.assertEqual(result['ready'],READY_ANGLES)
        self.assertGreater(result['cycle']['drop'][2],-110.0)

    def test_full_cycle_default_return_needs_no_return_callback(self):
        client=self.client()
        with patch('builtins.input',side_effect=AssertionError('unexpected return prompt')):
            result=self.run_arm(client,lambda *args:True,None)
        self.assertEqual(result['ready'],READY_ANGLES)

    def test_block_placement_cancel_stops_before_descent_and_closure(self):
        client=self.client()
        ret=Mock()
        with self.assertRaisesRegex(RuntimeError,'Cancelled at raised pickup'):
            self.run_arm(client,lambda *args:False,ret)
        self.assertEqual(client.send_angles.call_count,3)
        client.send_angle.assert_not_called()
        self.assertEqual([call.args[0] for call in client.set_gripper_state.call_args_list],[0])
        ret.assert_not_called()
        self.assertTrue(client.stop.called)

    def test_wrong_start_or_future_limit_blocks_before_any_command(self):
        for wrong_start in (True,False):
            client=self.client()
            if wrong_start:
                client.get_angles.return_value=list(VERIFIED_RAISED_ANGLES)
            else:
                client.get_joint_max_angle.side_effect=lambda joint:80 if joint==1 else 180
            with self.subTest(wrong_start=wrong_start),self.assertRaises((RuntimeError,ValueError)):
                self.run_arm(client)
            client.set_gripper_state.assert_not_called()
            client.send_angles.assert_not_called()
            client.send_angle.assert_not_called()

    def test_approach_fault_never_continues_to_grasp_or_return(self):
        client=self.client()
        original=client.send_angles.side_effect
        def fault(command,speed):
            if client.send_angles.call_count==2:
                client.get_error_information.return_value=1
            return original(command,speed)
        client.send_angles.side_effect=fault
        pickup,ret=Mock(),Mock()
        with self.assertRaises(RuntimeError):
            self.run_arm(client,pickup,ret)
        self.assertEqual(client.send_angles.call_count,2)
        pickup.assert_not_called()
        ret.assert_not_called()
        client.send_angle.assert_not_called()
        self.assertEqual([call.args[0] for call in client.set_gripper_state.call_args_list],[0])

    def test_missing_callbacks_refuse_before_queries(self):
        client=Mock()
        with self.assertRaisesRegex(RuntimeError,'requires block-placement'):
            RealArm(client).run_ready_cycle(None,None)
        self.assertEqual(client.mock_calls,[])

    def test_preplaced_mode_runs_without_any_operator_input(self):
        client=self.client()
        with patch('builtins.input',side_effect=AssertionError('unexpected prompt')):
            result=self.run_arm(client,preplaced_pickup,None)
        self.assertEqual(result['ready'],READY_ANGLES)
        self.assertEqual(client.send_angles.call_count,5)

    def test_cli_default_selects_preplaced_mode_and_keeps_manual_option(self):
        arm=Mock()
        arm.run_ready_cycle.return_value={'ready':READY_ANGLES}
        with patch('bucke_arm.full_cycle.RealArm',return_value=arm):
            with contextlib.redirect_stdout(io.StringIO()),patch('builtins.input',side_effect=AssertionError('unexpected prompt')):
                self.assertEqual(main(['--execute']),0)
                self.assertIs(arm.run_ready_cycle.call_args.args[0],preplaced_pickup)
                self.assertTrue(arm.run_ready_cycle.call_args.args[0](VERIFIED_RAISED_ANGLES))
                self.assertEqual(main(['--execute','--place-at-pickup']),0)
                self.assertIs(arm.run_ready_cycle.call_args.args[0],confirm_pickup)

    def test_default_and_fake_never_connect(self):
        with patch('bucke_arm.full_cycle.RealArm') as constructor:
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main([]),0)
                self.assertEqual(main(['--fake']),0)
        constructor.assert_not_called()
        arm=FakeArm(READY_ANGLES)
        self.assertEqual(arm.run_ready_cycle()['ready'],READY_ANGLES)


if __name__=='__main__':
    unittest.main()
