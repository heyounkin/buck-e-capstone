"""Exercise hover recovery without connecting to hardware."""

import contextlib
import io
from itertools import count
import unittest
from unittest.mock import Mock, patch

from bucke_arm.arm import READY_ANGLES, RealArm
from bucke_arm.resume_hover import HOVER, LIFT3_READBACK, main, return_route, route


class HoverTests(unittest.TestCase):

    def client(self):
        client = Mock()
        client.get_angles.return_value = list(READY_ANGLES)
        client.is_moving.return_value = 0
        client.get_error_information.return_value = 0
        client.get_joint_min_angle.return_value = -180
        client.get_joint_max_angle.return_value = 180
        client.send_angles.side_effect = lambda angles, speed: setattr(
            client.get_angles, 'return_value', list(angles))
        client.sync_send_angles.side_effect = lambda angles, speed, timeout: setattr(
            client.get_angles, 'return_value', list(angles))
        return client

    def test_synchronous_forward_uses_all_joint_calls_and_verifies_endpoint(self):
        client = self.client()
        self.assertTrue(self.run_arm(client, synchronous=True))
        self.assertEqual(client.sync_send_angles.call_count, 48)
        client.send_angles.assert_not_called()
        self.assertEqual(client.get_angles.return_value, list(HOVER))

    def test_planner_matches_independent_joint_progress_on_every_segment(self):
        points = [READY_ANGLES] + [q for _, q, _ in route()]
        for index, (start, end) in enumerate(zip(points, points[1:])):
            for fractions in ((0.25,) * 6, (0.75,) * 6, (0, 1, 0.3, 0.8, 0.5, 0.1)):
                with self.subTest(segment=index, progress=fractions):
                    actual = [a + f * (b - a) for a, b, f in zip(start, end, fractions)]
                    plan = return_route(actual)
                    if plan:
                        self.assertEqual(plan[-1][1], READY_ANGLES)
                        # All subsequent targets follow the original prefix backwards.
                        first = points.index(plan[0][1])
                        self.assertEqual([q for _, q, _ in plan], list(reversed(points[:first + 1])))

    def test_sync_success_cannot_override_an_expired_polling_deadline(self):
        client = self.client()
        client.sync_send_angles.side_effect = None
        client.sync_send_angles.return_value = 1
        with patch('bucke_arm.arm.time.monotonic', side_effect=[0, 9]):
            with self.assertRaisesRegex(RuntimeError, 'Timed out'):
                self.run_arm(client, synchronous=True)
        self.assertEqual(client.sync_send_angles.call_count, 1)
        client.stop.assert_called_once()

    def test_sync_exception_requests_stop_and_never_continues(self):
        client = self.client()
        client.sync_send_angles.side_effect = OSError('serial failure')
        with self.assertRaises(OSError):
            self.run_arm(client, synchronous=True)
        client.stop.assert_called_once()
        self.assertEqual(client.sync_send_angles.call_count, 1)

    def run_arm(self, client, **kwargs):
        with patch('bucke_arm.arm.time.sleep'), contextlib.redirect_stdout(io.StringIO()):
            return RealArm(client).move_to_hover(**kwargs)

    def test_resume_retries_lift3_then_only_remaining_route(self):
        client = self.client()
        client.get_angles.return_value = list(LIFT3_READBACK)
        self.assertTrue(self.run_arm(client, resume_lift3=True))
        self.assertEqual([c.args[0] for c in client.send_angles.call_args_list],
                         [list(target) for _, target, _ in route()[32:]])
        self.assertEqual(client.send_angles.call_count, 16)

    def test_resume_refuses_ready_instead_of_joining_route(self):
        client = self.client()
        with self.assertRaisesRegex(RuntimeError, 'recorded Lift 3'):
            self.run_arm(client, resume_lift3=True)
        client.send_angles.assert_not_called()

    def test_resume_still_stops_on_invalid_telemetry(self):
        client = self.client()
        client.get_angles.return_value = list(LIFT3_READBACK)
        client.send_angles.side_effect = lambda *args: setattr(
            client.get_angles, 'return_value', [])
        with self.assertRaises(ValueError):
            self.run_arm(client, resume_lift3=True)
        self.assertEqual(client.send_angles.call_count, 1)
        client.stop.assert_called_once()

    def test_preview_does_not_connect(self):
        with patch('bucke_arm.resume_hover.RealArm') as arm:
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main([]), 0)
            arm.assert_not_called()

    def test_wrong_start_never_moves(self):
        client = self.client()
        client.get_angles.return_value = [0] * 6
        with self.assertRaisesRegex(RuntimeError, 'saved ready'):
            self.run_arm(client)
        client.send_angles.assert_not_called()

    def test_fault_never_moves(self):
        client = self.client()
        client.get_error_information.return_value = 4
        with self.assertRaises(RuntimeError):
            self.run_arm(client)
        client.send_angles.assert_not_called()

    def test_full_route_has_no_descent_after_hover(self):
        client = self.client()
        self.assertTrue(self.run_arm(client))
        self.assertEqual(client.send_angles.call_count, 48)
        self.assertEqual(client.get_angles.return_value, list(HOVER))
        self.assertEqual([c.args[0] for c in client.send_angles.call_args_list],
                         [list(target) for _, target, _ in route()])

    def test_telemetry_loss_stops_without_continuing(self):
        client = self.client()
        client.send_angles.side_effect = lambda *args: setattr(
            client.get_angles, 'return_value', None)
        with self.assertRaises(ValueError):
            self.run_arm(client)
        self.assertEqual(client.send_angles.call_count, 1)
        client.stop.assert_called_once()

    def test_trim_is_bounded_and_stops_if_stalled(self):
        client = self.client()
        def move(angles, speed):
            actual = list(angles)
            if client.send_angles.call_count >= 48:
                actual = list(HOVER)
                actual[3] -= 1.8
            client.get_angles.return_value = actual
        client.send_angles.side_effect = move
        with patch('bucke_arm.arm.time.monotonic', side_effect=count(step=0.5)), \
                self.assertRaisesRegex(RuntimeError, 'Timed out at Hover trim'):
            self.run_arm(client)
        self.assertEqual(client.send_angles.call_count, 49)
        target = client.send_angles.call_args.args[0]
        self.assertAlmostEqual(target[3] - HOVER[3], 0.5)
        client.stop.assert_called_once()

    def test_cancel_closes_without_movement(self):
        client = self.client()
        with patch('bucke_arm.resume_hover.RealArm', return_value=RealArm(client)):
            with patch('builtins.input', return_value=''), patch('bucke_arm.arm.time.sleep'):
                with contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(main(['--execute']), 0)
        client.send_angles.assert_not_called()
        client._serial_port.close.assert_called_once()


if __name__ == '__main__':
    unittest.main()
