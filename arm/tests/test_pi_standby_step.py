"""Verify combined check/move/report behavior without hardware."""

import contextlib
import io
import itertools
import unittest
from unittest.mock import Mock, patch

import pi_standby_step as step


def client():
    arm = Mock()
    arm.get_system_version.return_value = 7.3
    arm.is_free_mode.return_value = 0
    arm.is_servo_enable.return_value = 1
    arm.is_moving.return_value = 0
    arm.get_error_information.return_value = 0
    arm.get_servo_status.return_value = [0] * 6
    arm.read_next_error.return_value = [0] * 6
    arm._serial_port.isOpen.return_value = True
    arm._serial_port.write.return_value = len(step.PACKET)
    arm._read.return_value = b""
    arm.get_angles.return_value = step.EXPECTED.copy()
    arm.get_servo_temps.return_value = [35, 33, 29, 38, 49, 33]
    arm.send_angle.side_effect = lambda *a, **kw: arm._serial_port.write(step.PACKET)
    return arm


class StandbyTests(unittest.TestCase):
    def setUp(self):
        self.output = io.StringIO()
        redirect = contextlib.redirect_stdout(self.output)
        redirect.__enter__()
        self.addCleanup(redirect.__exit__, None, None, None)
        for target, name, kwargs in [(step.time, "sleep", {}),
                                      (step.time, "monotonic", {"side_effect": itertools.count()})]:
            p = patch.object(target, name, **kwargs)
            p.start()
            self.addCleanup(p.stop)

    def test_preview_never_connects(self):
        sdk = Mock()
        with patch.dict("sys.modules", {"pymycobot": sdk}):
            self.assertEqual(step.main([]), 0)
        sdk.MyCobot280.assert_not_called()

    def test_cancel_never_sends_movement(self):
        arm = client()
        self.assertFalse(step.run(arm, lambda _: "cancel"))
        arm.send_angle.assert_not_called()
        arm.stop.assert_not_called()

    def test_combines_settled_endpoint_report_with_one_fixed_command(self):
        for endpoint in (67.33, 68.2):
            with self.subTest(endpoint=endpoint):
                arm = client()
                final = step.EXPECTED.copy()
                final[4] = endpoint
                arm.get_angles.side_effect = [step.EXPECTED] * 4 + [final] * 4
                with patch.object(step.time, "monotonic", side_effect=itertools.count()):
                    self.assertTrue(step.run(arm, lambda _: "MOVE J5"))
                arm.send_angle.assert_called_once_with(5, 67.33, 10, _async=True)
                arm._serial_port.write.assert_called_once_with(bytes.fromhex("fe fe 06 21 05 1a 4d 0a fa"))
                arm.stop.assert_not_called()
        self.assertIn("within +/-0.25: False", self.output.getvalue())
        self.assertIn("not an accuracy", self.output.getvalue())

    def test_unreviewed_pose_fault_temperature_and_enable_block_move(self):
        cases = [("get_angles", [108.1, -19.59, 11.07, 153.54, 72.33, -16.78]),
                 ("get_angles", [108.1, -19.59, 11.07, 146.42, 71.0, -16.78]),
                 ("get_angles", [float("nan")] * 6),
                 ("get_error_information", 4), ("get_error_information", False),
                 ("is_moving", 1), ("is_servo_enable", 0),
                 ("is_free_mode", 1), ("read_next_error", [0] * 6 + [1]),
                 ("get_servo_temps", [35, 33, 29, 38, -1, 33])]
        for name, value in cases:
            with self.subTest(name=name, value=value):
                arm = client()
                getattr(arm, name).return_value = value
                with self.assertRaises(RuntimeError):
                    step.run(arm, lambda _: "MOVE J5")
                arm.send_angle.assert_not_called()
                arm.stop.assert_not_called()

    def test_state_change_after_prompt_blocks_move(self):
        arm = client()
        changed = step.EXPECTED.copy()
        changed[0] += .3
        arm.get_angles.side_effect = [step.EXPECTED] * 3 + [changed]
        with self.assertRaisesRegex(RuntimeError, "State changed"):
            step.run(arm, lambda _: "MOVE J5")
        arm.send_angle.assert_not_called()

    def test_other_joint_motion_and_overshoot_stop(self):
        for joint, delta in [(0, .6), (3, .6), (5, -.6), (4, .4), (4, -5.4)]:
            arm = client()
            changed = step.EXPECTED.copy()
            changed[joint] += delta
            arm.get_angles.side_effect = [step.EXPECTED] * 4 + [changed]
            with self.subTest(joint=joint, delta=delta), self.assertRaises(RuntimeError):
                step.run(arm, lambda _: "MOVE J5")
            arm.stop.assert_called_once()

    def test_no_motion_does_not_count_as_settling_success(self):
        arm = client()
        with self.assertRaisesRegex(RuntimeError, "not confirmed"):
            step.run(arm, lambda _: "MOVE J5")
        arm.send_angle.assert_called_once()
        arm.stop.assert_called_once()

    def test_short_wrong_duplicate_and_missing_write_stop_and_restore(self):
        for failure in ("short", "wrong", "duplicate", "none", "interrupt"):
            with self.subTest(failure=failure):
                arm = client()
                port = arm._serial_port
                if failure == "short":
                    port.write.return_value = 4
                elif failure == "wrong":
                    arm.send_angle.side_effect = lambda *a, **kw: arm._serial_port.write(b"wrong")
                elif failure == "duplicate":
                    def duplicate(*args, **kwargs):
                        arm._serial_port.write(step.PACKET)
                        arm._serial_port.write(step.PACKET)
                    arm.send_angle.side_effect = duplicate
                elif failure == "none":
                    arm.send_angle.side_effect = None
                else:
                    arm.send_angle.side_effect = KeyboardInterrupt()
                with self.assertRaises((RuntimeError, KeyboardInterrupt)):
                    step.run(arm, lambda _: "MOVE J5")
                arm.stop.assert_called_once()
                self.assertIs(arm._serial_port, port)
                self.assertLessEqual(port.write.call_count, 1)

    def test_invalid_telemetry_after_write_stops(self):
        for fault in ("temperature", "angles"):
            arm = client()
            if fault == "temperature":
                arm.get_servo_temps.side_effect = [[35, 33, 29, 38, 49, 33]] * 4 + [[35, 33, 29, 38, -1, 33]]
            else:
                arm.get_angles.side_effect = [step.EXPECTED] * 4 + [-1]
            with self.subTest(fault=fault), self.assertRaises(RuntimeError):
                step.run(arm, lambda _: "MOVE J5")
            arm.stop.assert_called_once()

    def phase_client(self):
        arm = client()
        current = step.ZERO_START.copy()
        arm.get_angles.side_effect = lambda: current.copy()
        def move(joint, target, speed, **kwargs):
            arm._serial_port.write(step.packet_for(target))
            current[4] = target + 1.0  # Report shortfall without claiming accuracy.
        arm.send_angle.side_effect = move
        return arm

    def test_entire_j5_phase_has_exact_targets_and_one_confirmation_each(self):
        arm = self.phase_client()
        confirm = Mock(return_value="MOVE J5")
        self.assertTrue(step.run_toward_zero(arm, confirm))
        self.assertEqual(confirm.call_count, len(step.ZERO_TARGETS))
        self.assertEqual([c.args for c in arm.send_angle.call_args_list],
                         [(5, target, 10) for target in step.ZERO_TARGETS])
        self.assertEqual(arm._serial_port.write.call_count, len(step.ZERO_TARGETS))
        arm.stop.assert_not_called()
        self.assertIn("not the full standby", self.output.getvalue())
        self.assertIn("within +/-0.25: False", self.output.getvalue())

    def test_cancel_at_second_checkpoint_cancels_all_remaining_targets(self):
        arm = self.phase_client()
        confirm = Mock(side_effect=["MOVE J5", "cancel"])
        self.assertFalse(step.run_toward_zero(arm, confirm))
        arm.send_angle.assert_called_once_with(5, step.ZERO_TARGETS[0], 10, _async=True)
        arm.stop.assert_not_called()

    def test_new_diagnostic_before_next_step_prevents_remaining_targets(self):
        arm = self.phase_client()
        arm.read_next_error.side_effect = [[0] * 7, [0] * 7, [0, 0, 0, 0, 1, 0, 0]]
        with self.assertRaises(RuntimeError):
            step.run_toward_zero(arm, lambda _: "MOVE J5")
        arm.send_angle.assert_called_once()

    def test_progression_gate_does_not_accept_large_shortfall(self):
        final = step.ZERO_START.copy()
        final[4] = step.ZERO_TARGETS[0] + 1.3
        with patch.object(step, "run", return_value=final) as run:
            with self.assertRaisesRegex(RuntimeError, "progression gate"):
                step.run_toward_zero(client(), lambda _: "MOVE J5")
        run.assert_called_once()

    def test_other_joint_reference_is_not_rebased_during_phase(self):
        references = []
        def simulated(arm, confirm, expected, target):
            references.append(expected.copy())
            final = expected.copy()
            final[0] += .1
            final[4] = target + 1
            return final
        with patch.object(step, "run", side_effect=simulated):
            self.assertTrue(step.run_toward_zero(client()))
        self.assertTrue(all(r[0] == step.ZERO_START[0] for r in references))
        self.assertEqual(references[1][4], step.ZERO_TARGETS[0]+1)

    def test_cumulative_other_joint_drift_aborts_motion(self):
        arm = client()
        start = step.ZERO_START.copy()
        start[0] += .4
        changed = start.copy()
        changed[0] += .2  # Under per-move gate; over full-phase reference gate.
        arm.get_angles.side_effect = [start] * 4 + [changed]
        arm.send_angle.side_effect = lambda *a, **kw: arm._serial_port.write(step.packet_for(step.ZERO_TARGETS[0]))
        with self.assertRaisesRegex(RuntimeError, "Another joint"):
            step.run(arm, lambda _: "MOVE J5", expected=step.ZERO_START,
                     target=step.ZERO_TARGETS[0])
        arm.stop.assert_called_once()

    def test_targets_outside_fixed_plan_never_touch_hardware(self):
        for target in (True, float("nan"), -1, 10, 100):
            arm = client()
            with self.subTest(target=target), self.assertRaises(ValueError):
                step.run(arm, target=target)
            self.assertEqual(arm.mock_calls, [])

    def test_phase_preview_never_connects(self):
        sdk = Mock()
        with patch.dict("sys.modules", {"pymycobot": sdk}):
            self.assertEqual(step.main(["--toward-zero"]), 0)
        sdk.MyCobot280.assert_not_called()

    def test_temperature_above_previous_reading_is_reported_without_invented_cutoff(self):
        arm = self.phase_client()
        arm.get_servo_temps.return_value = [35, 33, 29, 38, 55, 33]
        self.assertTrue(step.run_toward_zero(arm, lambda _: "MOVE J5"))
        self.assertIn("55", self.output.getvalue())
        self.assertEqual(arm.send_angle.call_count, len(step.ZERO_TARGETS))

    def test_controller_servo_fault_still_aborts_after_write(self):
        arm = client()
        arm.get_servo_status.side_effect = [[0] * 6] * 4 + [[0, 0, 0, 0, 1, 0]]
        with self.assertRaisesRegex(RuntimeError, "fault"):
            step.run(arm, lambda _: "MOVE J5")
        arm.stop.assert_called_once()
        arm.send_angle.assert_called_once()


if __name__ == "__main__":
    unittest.main()
