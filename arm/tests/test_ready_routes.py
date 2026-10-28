"""Offline bounds, supported-start, and multi-waypoint preview checks."""

import contextlib
from dataclasses import asdict
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from bucke_arm.__main__ import main
from bucke_arm.ready import READY_ANGLES_DEG, move_to_ready
from bucke_arm.ready_config import (
    DEFAULT_LIMITS_PATH, JointLimits, ReadyRoute, load_limits, load_route,
)


# Resolve the bundled synthetic route independently of the terminal directory.
EXAMPLE_ROUTE = DEFAULT_LIMITS_PATH.with_name("ready_route.example.json")


class ReadyRouteTests(unittest.TestCase):
    def preview(self, pose, **kwargs):
        # **kwargs lets each case substitute limits or a route with fake status.
        return move_to_ready(pose, moving=0, error_information=0, **kwargs)

    def test_each_joint_limit_is_inclusive_and_rejects_values_just_outside(self):
        # Test both endpoints on every joint, then move 0.01 degree beyond each.
        limits = load_limits()
        for joint in range(6):
            for bound, outside in [(limits.minimum_deg[joint], -.01),
                                   (limits.maximum_deg[joint], .01)]:
                pose = list(READY_ANGLES_DEG)
                pose[joint] = bound
                self.preview(pose)
                pose[joint] += outside
                with self.subTest(joint=joint, bound=bound):
                    with self.assertRaisesRegex(ValueError, "Current pose J%d" % (joint + 1)):
                        self.preview(pose)

    def test_default_limits_are_explicitly_provisional_and_not_a_route(self):
        # Passing numeric bounds must not imply reviewed geometry or a chosen path.
        result = self.preview(READY_ANGLES_DEG)

        self.assertFalse(result.configured_limits.verified)
        self.assertFalse(result.joint_limits_verified)
        self.assertIn("v4.0.6", result.configured_limits.source)
        self.assertIsNone(result.route)
        self.assertEqual(result.waypoint_steps, ())

    def test_custom_limits_check_ready_target_even_if_start_is_inside(self):
        # Lower J5's allowed maximum below ready while leaving the start admissible.
        limits = JointLimits((-180,) * 6, (180, 180, 180, 180, 80, 180), "Test fixture")
        with self.assertRaisesRegex(ValueError, "Ready target J5"):
            self.preview([0] * 6, limits=limits)

    def test_two_waypoint_segments_use_previous_endpoint_and_finish_at_ready(self):
        # The synthetic route specifies one intermediate pose; ready is appended.
        # Segment two must start at segment one's endpoint, not the original pose.
        route = load_route(EXAMPLE_ROUTE)
        current = [90, 0, 0, 0, 90, -130]
        original = current.copy()
        result = self.preview(current, route=route)
        first, last = result.waypoint_steps

        self.assertEqual((first.step, last.step), (1, 2))
        self.assertEqual(first.from_angles_deg, tuple(current))
        self.assertEqual(first.to_angles_deg, (90, 0, 0, 0, 90, -132))
        self.assertEqual(last.from_angles_deg, first.to_angles_deg)
        self.assertEqual(last.to_angles_deg, READY_ANGLES_DEG)
        self.assertEqual(first.joint_changes[5].delta_deg, -2)
        self.assertEqual(last.joint_changes[5].delta_deg, -3)
        self.assertEqual(current, original)
        self.assertFalse(result.route_reviewed)
        self.assertFalse(result.hardware_execution_enabled)

    def test_each_joint_must_match_selected_start_range(self):
        # A pose may be within servo limits yet outside this route's starting box.
        route = load_route(EXAMPLE_ROUTE)
        for index in range(6):
            for endpoint, delta in [(route.start_min_deg, -.01), (route.start_max_deg, .01)]:
                pose = [90, 0, 0, 0, 90, -130]
                pose[index] = endpoint[index] + delta
                with self.subTest(index=index, delta=delta):
                    with self.assertRaisesRegex(ValueError, "does not support starting J%d" % (index + 1)):
                        self.preview(pose, route=route)

        self.preview(route.start_min_deg, route=route)
        self.preview(route.start_max_deg, route=route)

    def test_later_bad_waypoint_refuses_whole_preview(self):
        # A valid first waypoint cannot hide an out-of-bounds later destination.
        limits = load_limits()
        route = ReadyRoute("bad", "Test fixture", limits.minimum_deg,
                           limits.maximum_deg,
                           (READY_ANGLES_DEG, (0, 0, 151, 0, 90, -135)))
        with self.assertRaisesRegex(ValueError, "Waypoint 2 J3"):
            self.preview([0] * 6, route=route)

    def test_route_start_envelope_must_fit_limits_even_if_current_pose_fits(self):
        # Reject a route definition whose claimed coverage exceeds selected limits.
        route = ReadyRoute("wide", "Test fixture", (-180,) * 6, (180,) * 6, ())
        with self.assertRaisesRegex(ValueError, "Route start minimum J1"):
            self.preview(READY_ANGLES_DEG, route=route)

    def test_duplicate_waypoints_and_duplicate_ready_endpoint_are_skipped(self):
        # Repeated poses must not become extra candidate movement segments.
        limits = load_limits()
        start = (90, 0, 0, 0, 90, -130)
        route = ReadyRoute("duplicates", "Test fixture", limits.minimum_deg,
                           limits.maximum_deg, (start, start, READY_ANGLES_DEG, READY_ANGLES_DEG))
        result = self.preview(start, route=route)

        self.assertEqual(len(result.waypoint_steps), 1)
        self.assertEqual(result.waypoint_steps[0].to_angles_deg, READY_ANGLES_DEG)

    def test_already_ready_has_no_departure_even_with_supplied_waypoints(self):
        # Being at ready should not send the preview away just to follow waypoints.
        limits = load_limits()
        route = ReadyRoute("noop", "Test fixture", limits.minimum_deg, limits.maximum_deg,
                           ((90, 0, 0, 0, 90, -130),))
        result = self.preview(READY_ANGLES_DEG, route=route)

        self.assertTrue(result.exactly_at_target)
        self.assertEqual(result.waypoint_steps, ())

    def test_invalid_configuration_shapes_types_and_reversed_bounds(self):
        # Direct Python construction must enforce the same rules as JSON loading.
        # Each update changes one part of an otherwise valid configuration.
        data = asdict(load_limits())
        for update in [dict(minimum_deg=[0] * 5), dict(maximum_deg=[True] * 6),
                       dict(minimum_deg=[float("nan")] * 6),
                       dict(minimum_deg=[200] * 6), dict(source=""), dict(verified="false")]:
            with self.subTest(update=update), self.assertRaises(ValueError):
                JointLimits(**dict(data, **update))

        data = asdict(load_route(EXAMPLE_ROUTE))
        for update in [dict(start_min_deg=[0] * 6), dict(waypoints_deg=None),
                       dict(waypoints_deg=[[0] * 5]), dict(name=""),
                       dict(description=False)]:
            with self.subTest(update=update), self.assertRaises(ValueError):
                ReadyRoute(**dict(data, **update))

    def test_json_unknown_missing_and_malformed_fields_fail_clearly(self):
        # Temporary files exercise the real loader without altering project config.
        valid = asdict(load_limits())
        missing = dict(valid)
        del missing["minimum_deg"]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "limits.json"
            for data in [dict(valid, min_deg=[0] * 6), missing, [], None]:
                path.write_text(json.dumps(data), encoding="utf-8")
                with self.subTest(data=data), self.assertRaises(ValueError):
                    load_limits(path)

            path.write_text("{", encoding="utf-8")
            with self.assertRaises(ValueError):
                load_limits(path)

    def test_cli_config_errors_and_unsupported_start_have_no_partial_output(self):
        # File errors and route mismatch must fail cleanly before emitting JSON.
        base = ["move-to-ready", "--angles", "0", "0", "0", "0", "0", "0",
                "--moving", "0", "--error", "0"]
        with tempfile.TemporaryDirectory() as directory:
            missing = str(Path(directory) / "missing.json")
            for extra in [["--limits", missing], ["--route", missing],
                          ["--route", str(EXAMPLE_ROUTE)]]:
                output, errors = io.StringIO(), io.StringIO()
                with contextlib.redirect_stdout(output), contextlib.redirect_stderr(errors):
                    result = main(base + extra)

                self.assertEqual(result, 1)
                self.assertEqual(output.getvalue(), "")
                self.assertIn("Ready preview failed", errors.getvalue())

    def test_cli_route_and_custom_limits_remain_connection_free(self):
        # Even explicitly loaded route/limit files must not import the hardware
        # adapter or SDK. %r embeds Windows paths as valid Python string literals.
        code = (
            "import sys; from bucke_arm.__main__ import main; "
            "assert main(['move-to-ready', '--angles', '90', '0', '0', '0', '90', '-130', "
            "'--moving', '0', '--error', '0', '--route', %r, '--limits', %r]) == 0; "
            "assert not any(n == 'pymycobot' or n.startswith('pymycobot.') for n in sys.modules)"
        ) % (str(EXAMPLE_ROUTE), str(DEFAULT_LIMITS_PATH))
        result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)

        self.assertEqual(result.returncode, 0, result.stderr)
        preview = json.loads(result.stdout)

        self.assertEqual(len(preview["waypoint_steps"]), 2)
        self.assertFalse(preview["hardware_execution_enabled"])
        self.assertIn("synthetic", preview["route"]["name"])


if __name__ == "__main__":
    unittest.main()
