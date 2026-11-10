"""No hardware: capture integrity, exact sample reversal, and return lockout."""

import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from bucke_arm.arm import FakeArm, RealArm, READY_ANGLES
from bucke_arm.hover_trajectory import reverse_trace
from bucke_arm.resume_hover import HOVER, main


class TrajectoryTests(unittest.TestCase):
    def trace(self):
        middle = [(a + b) / 2 for a, b in zip(READY_ANGLES, HOVER)]
        return {"schema": 1, "direction": "ready_to_hover", "complete": True,
                "samples": [{"elapsed_s": t, "angles_deg": list(q), "moving": m, "error": 0}
                            for t, q, m in ((0.0, READY_ANGLES, 0), (0.2, middle, 1),
                                            (0.4, HOVER, 0))]}

    def reverse(self, data):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'trace.json'
            path.write_text(json.dumps(data))
            return reverse_trace(path)

    def test_reversal_preserves_every_sample_and_reverses_time(self):
        data = self.trace()
        result = self.reverse(data)
        self.assertEqual([p['angles_deg'] for p in result],
                         [p['angles_deg'] for p in reversed(data['samples'])])
        self.assertEqual([p['elapsed_s'] for p in result], [0.0, 0.2, 0.4])
        self.assertEqual(result[-1]['angles_deg'][5], -135)  # No shortest-angle wrapping.

    def test_incomplete_and_unreliable_recordings_are_refused(self):
        for kind in ('partial', 'gap', 'nonfinite', 'badtime', 'fault', 'wrongstart', 'wrongend'):
            data = self.trace()
            if kind == 'partial':
                data['complete'] = False
            elif kind == 'gap':
                data['samples'][2]['elapsed_s'] = 1.0
            elif kind == 'nonfinite':
                data['samples'][1]['angles_deg'][0] = float('nan')
            elif kind == 'badtime':
                data['samples'][1]['elapsed_s'] = 0.0
            elif kind == 'fault':
                data['samples'][1]['error'] = -1
            elif kind == 'wrongstart':
                data['samples'][0]['angles_deg'] = [0] * 6
            else:
                data['samples'][-1]['angles_deg'] = [0] * 6
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                self.reverse(data)

    def test_wrong_json_shapes_are_refused(self):
        data = self.trace()
        data["samples"][1] = None
        for malformed in ([], data):
            with self.subTest(data=malformed), self.assertRaises(ValueError):
                self.reverse(malformed)

    def test_cli_return_refuses_before_connecting(self):
        with patch('bucke_arm.resume_hover.RealArm') as constructor:
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main(['--execute', '--continuous', '--to-ready']), 1)
            constructor.assert_not_called()

    def test_real_and_fake_return_are_disabled_for_all_modes(self):
        client = Mock()
        for arm in (RealArm(client), FakeArm(HOVER)):
            for continuous in (False, True):
                with self.assertRaisesRegex(RuntimeError, 'disabled'):
                    arm.move_from_hover_to_ready(continuous=continuous)
        client.sync_send_angles.assert_not_called()
        client.send_angles.assert_not_called()

    def client(self):
        client = Mock()
        client.get_angles.return_value = list(READY_ANGLES)
        client.is_moving.return_value = 0
        client.get_error_information.return_value = 0
        def send(target, speed):
            client.get_angles.return_value = list(target)
            return 1
        client.send_angles.side_effect = send
        return client

    def test_forward_sends_one_command_and_saves_actual_samples(self):
        client = self.client()
        with patch('bucke_arm.arm.time.sleep'), patch('bucke_arm.hover_trajectory.save_forward_trace') as save:
            with contextlib.redirect_stdout(io.StringIO()):
                RealArm(client)._record_forward_hover(HOVER, 1.5)
        client.send_angles.assert_called_once_with(list(HOVER), 5)
        client.sync_send_angles.assert_not_called()
        samples, complete, failure = save.call_args.args
        self.assertTrue(complete)
        self.assertIsNone(failure)
        self.assertEqual(samples[0]['angles_deg'], list(READY_ANGLES))
        self.assertEqual(samples[-1]['angles_deg'], list(HOVER))
        self.assertEqual(len(samples), 4)

    def test_read_loss_stops_and_marks_recording_incomplete(self):
        client = self.client()
        def send(*args):
            client.get_angles.return_value = None
            return 1
        client.send_angles.side_effect = send
        with patch('bucke_arm.arm.time.sleep'), patch('bucke_arm.hover_trajectory.save_forward_trace') as save:
            with contextlib.redirect_stdout(io.StringIO()), self.assertRaises(ValueError):
                RealArm(client)._record_forward_hover(HOVER, 1.5)
        client.stop.assert_called_once()
        self.assertFalse(save.call_args.args[1])
        self.assertIsNotNone(save.call_args.args[2])


if __name__ == '__main__':
    unittest.main()
