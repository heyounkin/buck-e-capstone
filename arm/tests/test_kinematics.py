"""Offline geometry, orientation and CLI checks; never operate hardware."""

import contextlib
import io
import json
import math
import unittest
from unittest.mock import patch

from bucke_arm.arm import FakeArm
from bucke_arm.__main__ import main
from bucke_arm.kinematics import forward_kinematics, compare_xyz, _transform


class KinematicsTests(unittest.TestCase):
    def test_flange_local_pickup_point_rotates_with_wrist(self):
        offset = [6.35/math.sqrt(2), 6.35/math.sqrt(2), 100.0125]
        pose = forward_kinematics([90, 0, 0, 0, 90, -135])
        point = pose.point_in_base(offset)
        delta = tuple(point[i] - pose.xyz_mm[i] for i in range(3))
        for actual, expected in zip(delta, [-100.0125, 0, 6.35]):
            self.assertAlmostEqual(actual, expected, delta=0.001)
        turned = forward_kinematics([90, 0, 0, 0, 90, -45])
        turned_point = turned.point_in_base(offset)
        self.assertAlmostEqual(turned_point[2] - turned.xyz_mm[2], 0, delta=0.001)
        self.assertEqual(pose.point_in_base([0, 0, 0]), pose.xyz_mm)
        for bad in ([0, 0], [True]*3, [float('nan')]*3):
            with self.assertRaises(ValueError):
                pose.point_in_base(bad)

    def test_cli_tool_point_does_not_change_flange_comparison(self):
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.assertEqual(main(['fk', '--angles', '90', '0', '0', '0', '90', '-135',
                                   '--tool-offset-mm', '0', '0', '100',
                                   '--reported-xyz', '0', '0', '0']), 0)
        result = json.loads(output.getvalue())
        self.assertEqual(result['comparison']['predicted_xyz_mm'], result['xyz_mm'])
        self.assertAlmostEqual(result['pickup_point_base_mm'][0], result['xyz_mm'][0]-100)

    def test_reproduces_previously_documented_independent_calculation(self):
        # Rounded prediction recorded in standby_plan.md before this implementation.
        pose = forward_kinematics([108.01, -19.59, 11.07, 145.81, 76.28, -16.25])
        for actual, recorded in zip(pose.xyz_mm, [20.34, -0.06, 291.07]):
            self.assertAlmostEqual(actual, recorded, delta=0.005)
        comparison = compare_xyz([108.01, -19.59, 11.07, 145.81, 76.28, -16.25],
                                 [20.5, -1.3, 290.5])
        self.assertAlmostEqual(comparison['position_error_mm'], 1.38, delta=0.005)

    def test_base_joint_rotates_flange_about_base_z(self):
        angles = [0, -20, 30, 40, 50, 60]
        initial = forward_kinematics(angles)
        angles[0] = 90
        rotated = forward_kinematics(angles)
        x, y, z = initial.xyz_mm
        for actual, expected in zip(rotated.xyz_mm, [-y, x, z]):
            self.assertAlmostEqual(actual, expected, places=9)

    def test_last_joint_rotates_orientation_without_translating_flange_origin(self):
        first = forward_kinematics([0, 20, 30, 40, 50, 0])
        last = forward_kinematics([0, 20, 30, 40, 50, 90])
        self.assertEqual(first.xyz_mm, last.xyz_mm)
        for row, rotated in zip(first.rotation_matrix, last.rotation_matrix):
            for actual, expected in zip(rotated, (row[1], -row[0], row[2])):
                self.assertAlmostEqual(actual, expected, places=12)

    def test_orientation_is_proper_rotation_and_rpy_reconstructs_it(self):
        for angles in ([0]*6, [90, 0, 0, 0, 90, -135],
                       [12, -34, 56, -78, 90, -123]):
            pose = forward_kinematics(angles)
            r = pose.rotation_matrix
            for i in range(3):
                for j in range(3):
                    dot = sum(r[k][i]*r[k][j] for k in range(3))
                    self.assertAlmostEqual(dot, float(i == j), places=12)
            determinant = sum(r[0][i] * (r[1][(i+1)%3]*r[2][(i+2)%3]
                                         - r[1][(i+2)%3]*r[2][(i+1)%3]) for i in range(3))
            self.assertAlmostEqual(determinant, 1, places=12)
            reconstructed = _transform((0, 0, 0), tuple(map(math.radians, pose.rpy_deg)))
            for i in range(3):
                for j in range(3):
                    self.assertAlmostEqual(reconstructed[i][j], r[i][j], places=10)

    def test_rpy_at_both_gimbal_locks_reconstructs_rotation(self):
        from bucke_arm.kinematics import FlangePose
        for pitch in (-math.pi/2, math.pi/2):
            original = _transform((0, 0, 0), (0.4, pitch, -0.7))
            pose = FlangePose(original)
            reconstructed = _transform((0, 0, 0), tuple(map(math.radians, pose.rpy_deg)))
            for i in range(3):
                for j in range(3):
                    self.assertAlmostEqual(original[i][j], reconstructed[i][j], places=12)

    def test_invalid_angles_and_reported_coordinates(self):
        for bad in (None, [0]*5, [0]*7, [True]*6, ['0']*6,
                    [float('nan')]*6, [float('inf')]*6):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                forward_kinematics(bad)
        for bad in (None, [1, 2], [True]*3, [float('nan')]*3):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                compare_xyz([0]*6, bad)

    def test_fake_coordinates_follow_joint_commands_and_saved_poses(self):
        arm = FakeArm()
        first = arm.read_status()
        arm.move_to_ready()
        ready = arm.read_status()
        self.assertNotEqual(first.tool_pose_mm_deg[:3], ready.tool_pose_mm_deg[:3])
        self.assertEqual(ready.tool_pose_mm_deg, forward_kinematics(arm.get_angles()).pose_mm_deg)
        arm.move_joint(1, 0)
        self.assertNotEqual(ready.tool_pose_mm_deg[:3], arm.read_status().tool_pose_mm_deg[:3])
        arm.read_status().validate()

    def test_cli_comparison_stays_offline_and_rejects_nonfinite_values(self):
        output = io.StringIO()
        with patch('bucke_arm.__main__.RealArm') as real:
            with contextlib.redirect_stdout(output):
                self.assertEqual(main(['fk', '--angles', '0', '0', '0', '0', '0', '0',
                                       '--reported-xyz', '0', '0', '0']), 0)
            real.assert_not_called()
        result = json.loads(output.getvalue())
        self.assertEqual(result['frame'], 'base_to_flange')
        self.assertGreater(result['comparison']['position_error_mm'], 400)
        output, error = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(output), contextlib.redirect_stderr(error):
            self.assertEqual(main(['fk', '--angles', 'nan', '0', '0', '0', '0', '0']), 1)
        self.assertEqual(output.getvalue(), '')
        self.assertIn('FK failed', error.getvalue())


if __name__ == '__main__':
    unittest.main()
