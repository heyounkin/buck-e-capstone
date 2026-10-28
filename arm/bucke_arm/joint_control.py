
from pymycobot import MyCobot280
import math
import time

# myCobot 280 Pi
mc = MyCobot280("/dev/ttyAMA0", 1000000)

SPEED = 10
GRIPPER_SPEED = 20
POSE_TOLERANCE = 3.0
MOVE_TIMEOUT = 45

# Buck-E saved configurations (J1-J6)
STANDBY_ANGLES = [0, 0, 90, 90, 90, -130]
READY_ANGLES = [0, 0, 0, 0, 90, -130]

# Set True ONLY after physically validating
# the direct standby-to-ready motion.
READY_PATH_VERIFIED = False

SPEC_LIMITS = {
    1: (-168, 168),
    2: (-135, 135),
    3: (-150, 150),
    4: (-145, 145),
    5: (-155, 160),
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
            isinstance(robot_low, (int, float))
            and isinstance(robot_high, (int, float))
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

    if moving != 0 or error != 0:
        print(f"Movement blocked: moving={moving}, error={error}")
        return False

    return True


def matches_pose(current, target):
    return valid_angles(current) and all(
        abs(a - b) <= POSE_TOLERANCE
        for a, b in zip(current, target)
    )


def stop_arm():
    try:
        mc.stop()
    except Exception as exc:
        print(f"Software stop failed: {exc}")


def wait_for_target(target):
    """Verify movement completion and monitor controller faults."""
    start = time.monotonic()

    while time.monotonic() - start < MOVE_TIMEOUT:
        error = mc.get_error_information()
        moving = mc.is_moving()
        current = mc.get_angles()

        if error != 0 or moving not in (0, 1):
            print(f"Motion fault: moving={moving}, error={error}")
            stop_arm()
            return False

        if not valid_angles(current):
            print("Lost angle telemetry.")
            stop_arm()
            return False

        if moving == 0 and matches_pose(current, target):
            print("Target reached.")
            print("Final angles:", current)
            return True

        time.sleep(0.2)

    print("Movement timed out.")
    stop_arm()
    return False


def move_to_ready():
    print("\n--- READY TRANSITION ---")

    if not arm_ready():
        return

    current = mc.get_angles()

    if not valid_angles(current):
        print("Unable to read current angles.")
        return

    print("Current angles:", current)
    print("Required standby:", STANDBY_ANGLES)

    # Only allow transition from standby
    if not matches_pose(current, STANDBY_ANGLES):
        print("\nBLOCKED: Arm is not in standby position.")

        for i in range(6):
            difference = current[i] - STANDBY_ANGLES[i]
            print(f"J{i+1}: deviation {difference:+.2f} degrees")

        print("Move to standby manually first.")
        return

    print("\nStandby verified.")

    print("Target ready pose:", READY_ANGLES)

    # Validate target joint limits
    for joint, target in enumerate(READY_ANGLES, start=1):
        low, high = get_allowed_range(joint)

        if not low <= target <= high:
            print(f"J{joint} target violates joint limits.")
            return

    if not READY_PATH_VERIFIED:
        print("\nREADY MOVEMENT DISABLED.")
        print("Direct transition path has not been verified.")
        print("No movement commanded.")
        return

    print("\nWARNING: This moves multiple joints.")
    print("Verify the entire route is clear.")

    confirm = input("Type READY to execute: ").strip()

    if confirm != "READY":
        print("Movement cancelled.")
        return

    # Recheck immediately before issuing the command
    if not arm_ready():
        return

    latest = mc.get_angles()

    if not matches_pose(latest, STANDBY_ANGLES):
        print("Starting pose changed. Movement cancelled.")
        return

    print("\nMoving to ready position...")
    mc.send_angles(READY_ANGLES, SPEED)

    if wait_for_target(READY_ANGLES):
        print("READY POSITION CONFIRMED.")
    else:
        print("Ready transition failed. Inspect robot.")


print("\n=== BUCK-E WORKSPACE CONTROLLER ===")
print("1-6 : Move individual joint")
print("O   : Open gripper")
print("C   : Close gripper")
print("R   : Move from standby to ready")
print("Anything else : Quit\n")

try:
    while True:
        choice = input(
            "Select (1-6 / O / C / R): "
        ).strip().upper()

        # --- READY POSITION ---
        if choice == "R":
            move_to_ready()
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

print("Controller closed.")
