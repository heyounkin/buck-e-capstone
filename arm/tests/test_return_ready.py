"""Supervised return phase order, start/limit checks and cancellation."""

import contextlib
import io
import unittest
from unittest.mock import Mock,patch
from bucke_arm.arm import RealArm,FakeArm,READY_ANGLES
from bucke_arm.return_ready import DROP_ANGLES,STAGES,known_drop,targets,assignments,provisional_tool_height,main


class ReturnReadyTests(unittest.TestCase):
    def client(self):
        client=Mock()
        client.get_angles.return_value=list(DROP_ANGLES)
        client.is_moving.return_value=0
        client.get_error_information.return_value=0
        client.get_joint_min_angle.return_value=-180
        client.get_joint_max_angle.return_value=180
        def send(joint,value,speed):
            q=list(client.get_angles.return_value)
            q[joint-1]=value
            client.get_angles.return_value=q
            return 1
        client.send_angle.side_effect=send
        def send_all(target,speed):
            client.get_angles.return_value=list(target)
            return 1
        client.send_angles.side_effect=send_all
        return client

    def test_near_zero_j5_sign_does_not_reject_valid_drop(self):
        positive=list(DROP_ANGLES)
        positive[4]=.7
        self.assertTrue(known_drop(positive))
        positive[4]=3
        self.assertFalse(known_drop(positive))
        positive[4]=.7
        positive[1]+=2
        self.assertFalse(known_drop(positive))

    def test_route_raises_before_full_signed_wrist_turn_and_ends_ready(self):
        plan=targets()
        self.assertEqual([joint for _,joint,_ in plan],[2,6,3,4,(5,1)])
        self.assertEqual(plan[-1][2],READY_ANGLES)
        self.assertAlmostEqual(plan[1][2][5]-plan[0][2][5],-295.04,places=6)
        start=DROP_ANGLES
        for _,joint,end in plan:
            self.assertEqual(sum(a!=b for a,b in zip(start,end)),2 if joint==(5,1) else 1)
            heights=[provisional_tool_height(tuple(a+(b-a)*i/50 for a,b in zip(start,end))) for i in range(51)]
            self.assertGreater(min(heights),0)
            start=end

    def test_real_executes_single_joint_stages_only_after_callback(self):
        client=self.client()
        confirmations=[]
        def confirm(index,label,joint,current,target):
            self.assertEqual(client.send_angle.call_count,index-1)
            confirmations.append(joint)
            return True
        with patch('bucke_arm.arm.time.sleep'),patch('bucke_arm.hover_trajectory.save_forward_trace') as save:
            with contextlib.redirect_stdout(io.StringIO()):
                result=RealArm(client).return_to_ready_after_drop(confirm)
        self.assertEqual(result,READY_ANGLES)
        self.assertEqual(confirmations,[2,6,3,4,(5,1)])
        self.assertEqual([call.args for call in client.send_angle.call_args_list],
                         [(joint,value,10 if joint==6 else 5) for _,joint,value in STAGES if isinstance(joint,int)])
        self.assertEqual([call.kwargs.get("speed",5) for call in save.call_args_list],[5,10,5,5,5])
        client.send_angles.assert_called_once_with(list(READY_ANGLES),5)
        client.sync_send_angles.assert_not_called()
        client.set_gripper_state.assert_not_called()

    def test_default_return_does_not_request_confirmation(self):
        client=self.client()
        with patch('bucke_arm.arm.time.sleep'),patch('bucke_arm.hover_trajectory.save_forward_trace'),patch('builtins.input',side_effect=AssertionError('unexpected prompt')):
            with contextlib.redirect_stdout(io.StringIO()):
                result=RealArm(client).return_to_ready_after_drop()
        self.assertEqual(result,READY_ANGLES)
        client.send_angles.assert_called_once_with(list(READY_ANGLES),5)

    def test_invalid_callback_refuses_before_queries(self):
        client=Mock()
        with self.assertRaises(ValueError):
            RealArm(client).return_to_ready_after_drop('invalid')
        self.assertEqual(client.mock_calls,[])

    def test_cancellation_does_not_run_later_stages(self):
        client=self.client()
        with patch('bucke_arm.arm.time.sleep'),patch('bucke_arm.hover_trajectory.save_forward_trace'):
            with contextlib.redirect_stdout(io.StringIO()),self.assertRaisesRegex(RuntimeError,'Cancelled before return stage 2'):
                RealArm(client).return_to_ready_after_drop(lambda index,*args:index==1)
        client.send_angle.assert_called_once_with(2,0.0,5)
        client.stop.assert_called_once()
        client.set_gripper_state.assert_not_called()

    def test_future_limit_or_wrong_start_blocks_every_stage(self):
        for wrong_start in (True,False):
            client=self.client()
            if wrong_start:
                client.get_angles.return_value=list(READY_ANGLES)
            else:
                client.get_joint_min_angle.side_effect=lambda joint:10 if joint==4 else -180
            with self.subTest(wrong_start=wrong_start),patch('bucke_arm.arm.time.sleep'):
                with self.assertRaises((RuntimeError,ValueError)):
                    RealArm(client).return_to_ready_after_drop(lambda *args:True)
            client.send_angle.assert_not_called()

    def test_fault_stops_without_continuing_or_jaw_commands(self):
        client=self.client()
        def send(*args):
            client.get_error_information.return_value=1
            return 1
        client.send_angle.side_effect=send
        with patch('bucke_arm.arm.time.sleep'),patch('bucke_arm.hover_trajectory.save_forward_trace'):
            with contextlib.redirect_stdout(io.StringIO()),self.assertRaises(RuntimeError):
                RealArm(client).return_to_ready_after_drop(lambda *args:True)
        self.assertEqual(client.send_angle.call_count,1)
        self.assertTrue(client.stop.called)
        client.set_gripper_state.assert_not_called()

    def test_paired_command_reasserts_ready_goals_despite_prior_following_error(self):
        client=self.client()
        errors={2:-1.4,6:1.5,3:-.52,4:-.79}
        def send(joint,value,speed):
            q=list(client.get_angles.return_value)
            q[joint-1]=value+errors.get(joint,0)
            client.get_angles.return_value=q
            return 1
        client.send_angle.side_effect=send
        with patch('bucke_arm.arm.time.sleep'),patch('bucke_arm.hover_trajectory.save_forward_trace'):
            with contextlib.redirect_stdout(io.StringIO()):
                RealArm(client).return_to_ready_after_drop()
        client.send_angles.assert_called_once_with(list(READY_ANGLES),5)

    def test_paired_stage_fault_stops_without_reporting_ready(self):
        client=self.client()
        def send_all(*args):
            client.get_error_information.return_value=1
            return 1
        client.send_angles.side_effect=send_all
        with patch('bucke_arm.arm.time.sleep'),patch('bucke_arm.hover_trajectory.save_forward_trace'):
            with contextlib.redirect_stdout(io.StringIO()),self.assertRaises(RuntimeError):
                RealArm(client).return_to_ready_after_drop()
        self.assertEqual(client.send_angle.call_count,4)
        self.assertEqual(client.send_angles.call_count,1)
        self.assertTrue(client.stop.called)
        client.set_gripper_state.assert_not_called()

    def test_paired_stage_target_limit_preflight_blocks_all_motion(self):
        client=self.client()
        client.get_joint_min_angle.side_effect=lambda joint:-80 if joint==1 else -180
        with patch('bucke_arm.arm.time.sleep'),self.assertRaises(ValueError):
            RealArm(client).return_to_ready_after_drop()
        client.send_angle.assert_not_called()
        client.send_angles.assert_not_called()

    def test_default_preview_and_fake_never_open_hardware(self):
        with patch('bucke_arm.return_ready.RealArm') as constructor:
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main([]),0)
                self.assertEqual(main(['--fake']),0)
        constructor.assert_not_called()
        self.assertEqual(FakeArm(DROP_ANGLES).return_to_ready_after_drop(),READY_ANGLES)

    def test_old_hover_return_is_still_disabled(self):
        with self.assertRaisesRegex(RuntimeError,'disabled'):
            RealArm(Mock()).move_from_hover_to_ready()


if __name__=='__main__':
    unittest.main()
