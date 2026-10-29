
import argparse
import math
import time

# Open the hardware connection only when running main().
mc = None

SPEED = 10
GRIPPER_SPEED = 20
POSE_TOLERANCE = 3.0
SETTLED_SAMPLES = 3
SYNC_SPEED = 5
SYNC_TIMEOUT = 120
POST_SYNC_TIMEOUT = 5

# Buck-E saved configurations (J1-J6)
STANDBY_ANGLES = [0, 0, 90, 90, 90, -135]
READY_ANGLES = [-90, 0, 0, 0, 90, -135]

# User confirmed both synchronous transitions work safely on October 28, 2026.
# These flags record that report, not an independent hardware measurement.
READY_PATH_VERIFIED = True
STANDBY_PATH_VERIFIED = True

SPEC_LIMITS = {
    1: (-168, 168),
    2: (-135, 135),
    3: (-150, 150),
    4: (-145, 145),
    5: (-165, 165),
    6: (-180, 180),
}


def valid_angles(values):
    return (
        isinstance(values, (list, tuple))
        and len(values) == 6
        and all(
            isinstance(v, (int, float))
            and not isinstance(v, bool)
            and math.isfinite(v)
            for v in values
        )
    )


def get_allowed_range(joint):
    low, high = SPEC_LIMITS[joint]

    try:
        robot_low = mc.get_joint_min_angle(joint)
        robot_high = mc.get_joint_max_angle(joint)

        if (
            type(robot_low) in (int, float)
            and type(robot_high) in (int, float)
            and math.isfinite(robot_low)
            and math.isfinite(robot_high)
            and robot_low < robot_high
        ):
            print(
                f"Robot configured range: "
                f"{robot_low:.1f} to {robot_high:.1f}"
            )
            return max(low, robot_low), min(high, robot_high)

    except Exception:
        pass

    print("Robot limits unavailable; using API limits.")
    return low, high


def arm_ready():
    """Allow movement only when stationary and fault-free."""
    moving = mc.is_moving()
    error = mc.get_error_information()

    if type(moving) is not int or moving != 0 or type(error) is not int or error != 0:
        print(f"Movement blocked: moving={moving}, error={error}")
        return False

    return True


def matches_pose(current, target):
    return valid_angles(current) and valid_angles(target) and all(
        abs(a - b) <= POSE_TOLERANCE
        for a, b in zip(current, target)
    )


def stop_arm():
    try:
        mc.stop()
    except Exception as exc:
        print(f"Software stop failed: {exc}")


def wait_for_target(target, timeout=POST_SYNC_TIMEOUT):
    """Verify movement completion and monitor controller faults."""
    start = time.monotonic()
    settled = 0

    while time.monotonic() - start < timeout:
        error = mc.get_error_information()
        moving = mc.is_moving()
        current = mc.get_angles()

        if (type(error) is not int or error != 0
                or type(moving) is not int or moving not in (0, 1)):
            print(f"Motion fault: moving={moving}, error={error}")
            stop_arm()
            return False

        if not valid_angles(current):
            print("Lost angle telemetry.")
            stop_arm()
            return False

        settled = settled + 1 if moving == 0 and matches_pose(current, target) else 0
        if settled >= SETTLED_SAMPLES:
            print("Target reached within", POSE_TOLERANCE, "degrees per joint.")
            print("Final angles:", current)
            return True

        time.sleep(0.2)

    print("Movement timed out.")
    stop_arm()
    return False


def move_to_pose(name, start_name, start_pose, target_pose, path_verified):
    """Run the selected transition without a second confirmation prompt."""
    print(f"\n--- {name} TRANSITION ---")
    if not arm_ready():
        return False

    current = mc.get_angles()
    if not valid_angles(current):
        print("Unable to read current angles.")
        return False

    print("Current angles:", current)
    print(f"Required {start_name.lower()}:", start_pose)
    print(f"Target {name.lower()} pose:", target_pose)
    if matches_pose(current, target_pose):
        print(f"Already within {name.lower()}-pose tolerance. No movement commanded.")
        return True

    if not matches_pose(current, start_pose):
        print(f"BLOCKED: Arm is not in {start_name.lower()} position.")
        for joint, (angle, expected) in enumerate(zip(current, start_pose), 1):
            print(f"J{joint}: deviation {angle - expected:+.2f} degrees")
        return False

    for joint, target in enumerate(target_pose, start=1):
        low, high = get_allowed_range(joint)
        if not low <= target <= high:
            print(f"J{joint} target violates joint limits.")
            return False

    if not path_verified:
        print(f"{name} MOVEMENT DISABLED.")
        print(f"Direct {start_name.lower()}-to-{name.lower()} path has not been verified.")
        print("No movement commanded.")
        return False

    print("This moves multiple joints. Verify the entire route is clear.")
    print(f"Synchronous SDK move: speed {SYNC_SPEED}, timeout {SYNC_TIMEOUT} seconds.")
    if not arm_ready():
        return False
    latest = mc.get_angles()
    if not matches_pose(latest, start_pose):
        print("Starting pose changed. Movement cancelled.")
        return False

    print(f"Moving to {name.lower()} position...")
    try:
        # SDK 4.0.6 returns 1 even after its timeout. Verify actual telemetry
        # after it returns; the SDK blocks our own polling during this call.
        mc.sync_send_angles(target_pose, SYNC_SPEED, timeout=SYNC_TIMEOUT)
        reached = wait_for_target(target_pose, timeout=POST_SYNC_TIMEOUT)
    except BaseException:
        # A failed SDK call may still have transmitted the command.
        stop_arm()
        raise

    if reached:
        print(f"{name} POSITION CONFIRMED.")
    else:
        print(f"{name.title()} transition failed. Inspect robot.")
    return reached


def move_to_ready():
    return move_to_pose("READY", "STANDBY", STANDBY_ANGLES, READY_ANGLES,
                        READY_PATH_VERIFIED)


def move_to_standby():
    return move_to_pose("STANDBY", "READY", READY_ANGLES, STANDBY_ANGLES,
                        STANDBY_PATH_VERIFIED)


def main(argv=None):
    """Connect on the Pi and run the interactive controller."""
    global mc
    parser = argparse.ArgumentParser(description="Buck-E interactive hardware controller")
    parser.parse_args(argv)
    from pymycobot import MyCobot280

    mc = MyCobot280("/dev/ttyAMA0", 1000000)
    print("\n=== BUCK-E WORKSPACE CONTROLLER ===")
    print("1-6 : Move individual joint")
    print("O   : Open gripper")
    print("C   : Close gripper")
    print("R   : Move from standby to ready")
    print("S   : Move from ready to standby")
    print("Anything else : Quit\n")
    print("R/S: execute synchronous all-joint movement immediately, speed 5.")

    try:
        while True:
            choice = input(
                "Select (1-6 / O / C / R / S): "
            ).strip().upper()

            # --- READY POSITION ---
            if choice == "R":
                move_to_ready()
                continue

            if choice == "S":
                move_to_standby()
                continue

            # --- GRIPPER CONTROL ---
            if choice in ("O", "C"):
                if not arm_ready():
                    print("Check arm status before retrying.")
                    continue

                if choice == "O":
                    print("Opening gripper...")
                    mc.set_gripper_state(0, GRIPPER_SPEED)
                else:
                    print("Closing gripper...")
                    mc.set_gripper_state(1, GRIPPER_SPEED)

                print("Gripper command sent.\n")
                continue

            # --- EXIT ---
            if choice not in ("1", "2", "3", "4", "5", "6"):
                print("Exiting controller.")
                break

            # --- JOINT CONTROL ---
            joint = int(choice)
            current = mc.get_angles()

            if not valid_angles(current):
                print("Unable to read joint angles. Stopping.")
                break

            current_angle = current[joint - 1]
            spec_low, spec_high = SPEC_LIMITS[joint]

            print(f"\n--- JOINT {joint} ---")
            print(f"Current angle: {current_angle:.2f} degrees")
            print(f"API range: {spec_low} to {spec_high} degrees")

            low, high = get_allowed_range(joint)

            if low >= high:
                print("Invalid joint limits. Stopping.")
                break

            print(f"Allowed target range: {low:.1f} to {high:.1f}")

            try:
                target = float(input("Desired angle: "))
            except ValueError:
                print("Invalid angle. Try again.")
                continue

            if not math.isfinite(target) or not low <= target <= high:
                print("Target outside allowed range or invalid.")
                continue

            if not arm_ready():
                print("Check arm status before retrying.")
                continue

            change = target - current_angle

            print(f"\nJ{joint}: {current_angle:.2f} -> {target:.2f}")
            print(f"Angle change: {change:+.2f} degrees")

            confirm = input("Type MOVE to execute: ")

            if confirm.strip() != "MOVE":
                print("Movement cancelled.")
                continue

            # Recheck status before motion
            if not arm_ready():
                continue

            mc.send_angle(joint, target, SPEED)
            print("Command sent. Verify motion is complete.\n")

    except KeyboardInterrupt:
        print("\nInterrupted. Requesting stop.")
        stop_arm()
        try:
            mc.gripper_stop()
        except Exception as exc:
            print(f"Gripper stop failed: {exc}")

    finally:
        mc._serial_port.close()

    print("Controller closed.")


if __name__ == "__main__":
    main()
