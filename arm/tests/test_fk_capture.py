"""Exercise capture with a read-only SDK double; no real serial connection."""

import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from bucke_arm.arm import RealArm, STANDBY_ANGLES, READY_ANGLES
from bucke_arm.fk_capture import collect, main
from bucke_arm.kinematics import forward_kinematics


def client_at(angles=STANDBY_ANGLES):
    client = Mock(spec=['get_angles', 'get_coords', 'is_moving',
                        'get_error_information', 'get_reference_frame',
                        'get_end_type', 'get_tool_reference', 'get_world_reference',
                        '_serial_port'])
    client.get_angles.return_value = list(angles)
    client.get_coords.return_value = forward_kinematics(angles).pose_mm_deg
    client.is_moving.return_value = 0
    client.get_error_information.return_value = 0
    client.get_reference_frame.return_value = 0
    client.get_end_type.return_value = 0
    client.get_tool_reference.return_value = [0]*6
    client.get_world_reference.return_value = [0]*6
    return client


class CaptureTests(unittest.TestCase):
    def collect(self, client, pose='standby'):
        with patch('bucke_arm.fk_capture.time.sleep'):
            return collect(RealArm(client), pose)

    def test_three_samples_at_each_saved_pose_use_only_allowed_reads(self):
        for pose, angles in [('standby', STANDBY_ANGLES), ('ready', READY_ANGLES)]:
            with self.subTest(pose=pose):
                client = client_at(angles)
                report = self.collect(client, pose)
                self.assertTrue(report['capture_complete'])
                self.assertEqual(len(report['samples']), 3)
                self.assertEqual(report['position_error_mm']['max'], 0)
                self.assertEqual(client.get_angles.call_count, 6)
                self.assertEqual(client.get_reference_frame.call_count, 6)
                self.assertEqual(client.get_coords.call_count, 3)
                sample = report['samples'][0]
                self.assertEqual(sample['references_before'], sample['references_after'])

    def test_faults_invalid_references_and_malformed_data_refuse_comparison(self):
        for method, value in [('is_moving', 1), ('is_moving', False),
                              ('get_error_information', 4), ('get_error_information', None),
                              ('get_reference_frame', 1), ('get_reference_frame', False),
                              ('get_end_type', 1), ('get_tool_reference', [1]*6),
                              ('get_world_reference', [0]*5), ('get_coords', None),
                              ('get_angles', [float('nan')]*6)]:
            with self.subTest(method=method, value=value):
                client = client_at()
                getattr(client, method).return_value = value
                report = self.collect(client)
                self.assertFalse(report['capture_complete'])
                self.assertNotIn('position_error_mm', report)
                self.assertIn('failure', report)

    def test_frame_change_or_fault_after_query_is_detected(self):
        for method, values in [('get_end_type', [0, 1]),
                               ('get_error_information', [0, 4]),
                               ('is_moving', [0, 1])]:
            client = client_at()
            getattr(client, method).side_effect = values
            report = self.collect(client)
            self.assertFalse(report['capture_complete'])
            self.assertNotIn('position_error_mm', report)

    def test_wrong_pose_label_and_drift_within_or_between_samples(self):
        report = self.collect(client_at(READY_ANGLES))
        self.assertFalse(report['capture_complete'])
        self.assertIn('selected saved pose', report['failure'])
        shifted = list(STANDBY_ANGLES)
        shifted[0] += 0.3
        for readings in ([STANDBY_ANGLES, shifted],
                         [STANDBY_ANGLES]*2 + [shifted]*2):
            client = client_at()
            client.get_angles.side_effect = readings
            report = self.collect(client)
            self.assertFalse(report['capture_complete'])
            self.assertIn('0.25', report['failure'])

    def test_completed_samples_survive_later_query_failure(self):
        client = client_at()
        client.get_coords.side_effect = [client.get_coords.return_value, OSError('lost reply')]
        report = self.collect(client)
        self.assertFalse(report['capture_complete'])
        self.assertEqual(len(report['samples']), 1)
        self.assertIn('comparison', report['samples'][0])
        self.assertEqual(report['failure'], 'lost reply')

    def test_cli_saves_report_closes_port_and_never_overwrites(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'capture.json'
            client = client_at()
            with patch('bucke_arm.fk_capture.RealArm', return_value=RealArm(client)) as real:
                with patch('bucke_arm.fk_capture.time.sleep'), contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(main(['--pose', 'standby', '--output', str(output)]), 0)
                client._serial_port.close.assert_called_once()
                saved = output.read_text(encoding='utf-8')
                self.assertTrue(json.loads(saved)['capture_complete'])
                with contextlib.redirect_stderr(io.StringIO()):
                    self.assertEqual(main(['--pose', 'ready', '--output', str(output)]), 1)
                self.assertEqual(real.call_count, 1)
                self.assertEqual(output.read_text(encoding='utf-8'), saved)

    def test_cli_saves_failure_and_closes_after_bad_read(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'capture.json'
            client = client_at()
            client.get_coords.return_value = None
            with patch('bucke_arm.fk_capture.RealArm', return_value=RealArm(client)):
                with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                    self.assertEqual(main(['--pose', 'standby', '--output', str(output)]), 1)
            client._serial_port.close.assert_called_once()
            self.assertFalse(json.loads(output.read_text(encoding='utf-8'))['capture_complete'])


if __name__ == '__main__':
    unittest.main()
