"""Supervised J5 standby steps with local checkpoints; preview by default."""

import argparse
import math
import time

# Historical J5-only experiment, separate from the newer move_to_ready preview.
# These recorded poses are not automatically refreshed from the current arm.
EXPECTED = [108.10, -19.59, 11.07, 146.42, 72.33, -16.78]
TARGET = 67.33
SPEED = 10

# Expected SDK wire message for J5 at 67.33 degrees and speed setting 10.
PACKET = bytes.fromhex("fe fe 06 21 05 1a 4d 0a fa")

# The multi-step phase uses a different, later observed starting pose.
ZERO_START = [108.10, -19.59, 11.07, 146.42, 68.37, -16.78]
ZERO_TARGETS = (63.37, 58.37, 53.37, 48.37, 43.37, 38.37, 33.37,
                28.37, 23.37, 18.37, 13.37, 8.37, 3.37, 0.0)


def six_numbers(value, label):
    # Diagnostic arrays and joint angles must all have six finite numeric entries.
    if (not isinstance(value, (list, tuple)) or len(value) != 6
            or any(isinstance(x, bool) or not isinstance(x, (int, float))
                   or not math.isfinite(x) for x in value)):
        raise RuntimeError("Invalid " + label + ": " + repr(value))

    return list(value)


def packet_for(target):
    # Only the explicitly listed J5 targets may reach the command-writing code.
    if type(target) not in (int, float) or target not in (TARGET,) + ZERO_TARGETS:
        raise ValueError("Target is outside the fixed J5 plan")

    # Encode hundredths of a degree as high/low bytes. This fixed plan has
    # nonnegative targets; this is not a general signed-angle packet encoder.
    encoded = round(target * 100)
    return bytes([0xFE, 0xFE, 6, 0x21, 5, encoded >> 8, encoded & 255, SPEED, 0xFA])


def state(arm):
    # Query calls are sequential; a valid reply does not make them simultaneous.
    angles = six_numbers(arm.get_angles(), "angles")
    moving = arm.is_moving()
    error = arm.get_error_information()
    temps = six_numbers(arm.get_servo_temps(), "temperatures")
    status = six_numbers(arm.get_servo_status(), "servo status")
    if type(moving) is not int or moving not in (0, 1):
        raise RuntimeError("Invalid motion status")

    if type(error) is not int or error != 0 or any(x != 0 for x in status):
        raise RuntimeError("Controller/servo fault: error=%r status=%r" % (error, status))

    # Reject invalid negative temperature replies, without inventing a maximum
    # operating temperature from the previous session's observed readings.
    if any(t < 0 for t in temps):
        raise RuntimeError("Invalid temperature reply")

    return angles, moving, temps


def ready(arm):
    # This helper's name means preflight readiness, not the six-joint ready pose.
    # Require the known firmware, holding enabled, and clear diagnostic replies.
    if arm.get_system_version() != 7.3:
        raise RuntimeError("Expected firmware 7.3")

    free = arm.is_free_mode()
    enabled = [arm.is_servo_enable(j) for j in range(1, 7)]
    diag = arm.read_next_error()
    if (type(free) is not int or free != 0 or
            any(type(x) is not int or x != 1 for x in enabled) or
            not isinstance(diag, (list, tuple)) or len(diag) not in (6, 7) or
            any(type(x) is not int or x != 0 for x in diag)):
        raise RuntimeError("Free mode, enable state, or diagnostics unsuitable")


def send_once(arm, target=TARGET):
    # Temporarily intercept this SDK client's movement write. Serial operations
    # still use the original port, with extra checks on the outgoing packet.
    packet = packet_for(target)
    original = arm._serial_port

    class Port:
        # The instance increments this counter; a second write is refused.
        attempts = 0

        def __getattr__(self, name):
            # Delegate port settings/read methods that are not intercepted here.
            return getattr(original, name)

        def isOpen(self):
            # RuntimeError makes a closed port visible to our caller.
            if not original.isOpen():
                raise RuntimeError("Serial port is closed")

            return True

        def write(self, data):
            # The guard validates bytes before passing them to the real port.
            self.attempts += 1
            if self.attempts != 1 or bytes(data) != packet:
                raise RuntimeError("Blocked unexpected or repeated movement packet")

            try:
                count = original.write(data)
            except Exception as exc:
                raise RuntimeError("Movement write failed") from exc

            if type(count) is not int or count != len(packet):
                # A successful call must acknowledge every byte, not a partial write.
                raise RuntimeError("Incomplete movement write")

            return count

    port = Port()
    arm._serial_port = port
    try:
        arm.send_angle(5, target, SPEED, _async=True)
        if port.attempts != 1:
            raise RuntimeError("No movement write attempted")
    finally:
        # Restore the original port even when the movement call raises.
        arm._serial_port = original

    # Attempt to capture a raw reply before a later SDK query clears its buffer.
    # Neither a reply nor a full byte count proves physical target attainment.
    reply = arm._read(0x21, timeout=0.25)
    if not isinstance(reply, (bytes, bytearray)):
        raise RuntimeError("Invalid movement reply")

    print("TX:", packet.hex(" "), "RX:", bytes(reply).hex(" ") or "none", flush=True)


def run(arm, confirm=input, expected=None, target=TARGET):
    # A caller can inject confirmation and a client for offline tests. Hardware
    # execution still requires explicit selection in main and a local prompt.
    packet_for(target)  # Validate before hardware access.
    expected = list(EXPECTED if expected is None else expected)

    # Track command attempts, not acknowledgements: a failed call may have sent data.
    attempted = False
    try:
        ready(arm)
        baseline = None

        # Three preflight readings must remain near the reviewed starting pose.
        # j == 4 refers to J5 because Python list indices start at zero.
        for _ in range(3):
            angles, moving, temps = state(arm)
            if moving or any(abs(a-b) > (0.3 if j == 4 else 0.5)
                             for j, (a, b) in enumerate(zip(angles, expected))):
                raise RuntimeError("Starting pose differs from the last reported endpoint, or arm is moving")

            if baseline is not None and any(abs(a-b) > 0.2 for a, b in zip(angles, baseline)):
                raise RuntimeError("Starting pose has not settled")

            baseline = angles
            time.sleep(0.2)

        print("BEFORE:", angles, "Temperatures C:", temps, flush=True)
        if not 0 < baseline[4] - target <= 6.6:
            # This experiment only supports a small decrease, never a return move.
            raise RuntimeError("Target is not a small decrease from this starting pose")

        print("One J5 move to", target, "degrees, speed 10; next move requires a new prompt.", flush=True)
        print("Proceed only if only J5 physically moved last time; gripper detached, mount secure, cable/wrist gap clear, switch reachable.", flush=True)
        if confirm("Type MOVE J5 to authorize this step, or anything else to cancel: ").strip() != "MOVE J5":
            return False

        # Recheck after human input, which may take an arbitrarily long time.
        ready(arm)
        angles, moving, temps = state(arm)
        if moving or any(abs(a-b) > 0.2 for a, b in zip(angles, baseline)):
            raise RuntimeError("State changed while waiting; no command sent")

        baseline = angles

        # Set before sending so an exception during transmission requests stop.
        attempted = True
        started = time.monotonic()
        send_once(arm, target)
        previous = None
        stable = 0
        while time.monotonic() - started < 10:
            # Compare other joints against both this step's baseline and the
            # original phase reference to catch local and cumulative drift.
            angles, moving, temps = state(arm)
            if any(abs(angles[j]-baseline[j]) > 0.5 or abs(angles[j]-expected[j]) > 0.5
                   for j in (0, 1, 2, 3, 5)):
                raise RuntimeError("Another joint changed over 0.5 degrees")

            if not target-0.25 <= angles[4] <= baseline[4]+0.25:
                raise RuntimeError("J5 left the planned interval")

            # Count successive stationary transitions within the experimental
            # drift allowance. This is distinct from reaching the target angle.
            stable = stable + 1 if (not moving and previous is not None and
                      all(abs(a-b) <= 0.2 for a, b in zip(angles, previous))) else 0
            previous = angles
            if time.monotonic()-started >= 2 and stable >= 3 and baseline[4]-angles[4] >= 1:
                # Report settled movement and target error separately; a stable
                # shortfall is not silently treated as accurate positioning.
                print("AFTER:", angles, "Moving: 0 Error: 0 Temperatures C:", temps, flush=True)
                print("J5 target error: %+.2f degrees; within +/-0.25: %s" %
                      (angles[4]-target, abs(angles[4]-target) <= 0.25), flush=True)
                print("Stationary endpoint observed; this is not an accuracy or thermal-safety pass. No further command.", flush=True)
                return angles

            time.sleep(0.25)

        raise RuntimeError("Movement/settling not confirmed in observation window")
    except BaseException:
        # Include Ctrl+C in stop handling after an attempt, then re-raise the
        # original failure. Cancellation before an attempt sends no stop command.
        if attempted:
            print("STOP requested; stop preserves holding torque. Use the switch if unexpected motion continues.", flush=True)
            try:
                print("Stop reply:", arm.stop(), "Angles:", arm.get_angles(), flush=True)
            except BaseException as exc:
                print("Stop/read failed:", repr(exc), flush=True)

        raise


def run_toward_zero(arm, confirm=input):
    """Fixed targets only, with a fresh physical confirmation before each write."""

    # Copy constants so recording an endpoint cannot mutate the phase definition.
    expected = ZERO_START.copy()
    print("J5-only phase; all other joints remain at", ZERO_START, flush=True)
    print("Targets:", ZERO_TARGETS, flush=True)
    print("Inspect plate, wrist gap and cables throughout the arc. Cancel if clearance is uncertain.", flush=True)
    for number, target in enumerate(ZERO_TARGETS, 1):
        print("\nCHECKPOINT %d/%d" % (number, len(ZERO_TARGETS)), flush=True)
        final = run(arm, confirm, expected=expected, target=target)
        if final is False:
            print("Cancelled; remaining targets were not sent.", flush=True)
            return False

        if abs(final[4] - target) > 1.25:
            # This experimental progression gate is wider than the separately
            # reported 0.25-degree accuracy check; it does not redefine accuracy.
            raise RuntimeError("Settled endpoint exceeds experimental 1.25-degree progression gate; remaining targets cancelled")

        # Only J5's observed endpoint becomes the next starting reference.
        # Other-joint references stay fixed for the entire phase to catch
        # cumulative drift. Commanded targets are never calculated from readings.
        expected[4] = final[4]

    print("J5 PHASE FINISHED. Final angles:", final, flush=True)
    print("This is the wrist phase, not the full standby pose. No other joint will move.", flush=True)
    return True


def main(argv=None):
    # Preview is the default. The SDK is imported only after --hardware is chosen.
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--hardware", action="store_true")
    parser.add_argument("--toward-zero", action="store_true",
                        help="fixed J5-only sequence from reviewed 68.37-degree endpoint")
    args = parser.parse_args(argv)
    if args.toward_zero:
        print("J5-only phase: reviewed 68.37 degrees -> fixed targets", ZERO_TARGETS,
              "at speed 10, with a confirmation before EVERY move.")
        print("Settled observations permit progression only within 1.25 degrees of each target;")
        print("this experimental progression gate is separate from the unchanged +/-0.25 accuracy check.")
    else:
        print("Historical single step: reviewed 72.33 -> fixed 67.33 degrees, speed 10.")

    print("Temperatures are reported; controller/servo faults stop the sequence. No arbitrary temperature cutoff.")
    if not args.hardware:
        print("Preview only; no SDK import or connection.")
        return 0

    from pymycobot import MyCobot280, PI_PORT, PI_BAUD, __version__
    if (__version__, PI_PORT, PI_BAUD) != ("4.0.6", "/dev/ttyAMA0", 1000000):
        raise RuntimeError("Unexpected SDK or connection settings")

    arm = MyCobot280(PI_PORT, PI_BAUD)
    try:
        result = run_toward_zero(arm) if args.toward_zero else run(arm)
        return 0 if result else 1
    except (Exception, KeyboardInterrupt) as exc:
        print("Ended:", str(exc) or type(exc).__name__, flush=True)
        return 1
    finally:
        # Closing UART does not release torque or stop a moving robot.
        arm._serial_port.close()


if __name__ == "__main__":
    # Propagate main's success/failure as the shell's process exit code.
    raise SystemExit(main())
