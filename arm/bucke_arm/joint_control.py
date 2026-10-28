
from pymycobot import MyCobot280
import math

# myCobot 280 Pi
mc = MyCobot280("/dev/ttyAMA0", 1000000)

SPEED = 10

# Conservative limits from the myCobot 280 Python API
SPEC_LIMITS = {
    1: (-168, 168),
    2: (-135, 135),
    3: (-150, 150),
    4: (-145, 145),
    5: (-155, 160),
    6: (-180, 180),
}


def get_allowed_range(joint):
    """Use the intersection of API and robot limits."""
    low, high = SPEC_LIMITS[joint]

    try:
        robot_low = mc.get_joint_min_angle(joint)
        robot_high = mc.get_joint_max_angle(joint)

        if all(
            isinstance(x, (int, float))
            and not isinstance(x, bool)
            and math.isfinite(x)
            for x in (robot_low, robot_high)
        ) and robot_low < robot_high:
            print(
                f"Robot configured range: "
                f"{robot_low:.1f} to {robot_high:.1f} degrees"
            )
            return max(low, robot_low), min(high, robot_high)

    except Exception:
        pass

    print("Robot limits unavailable; using documented API limits.")
    return low, high


print("\n=== BUCK-E WORKSPACE JOINT CONTROLLER ===")
print("Enter joint 1-6. Anything else quits.\n")

try:
    while True:
        choice = input("Select joint (1-6): ").strip()

        if choice not in ("1", "2", "3", "4", "5", "6"):
            print("Exiting controller.")
            break

        joint = int(choice)

        current = mc.get_angles()

        if (
            not isinstance(current, (list, tuple))
            or len(current) != 6
            or not all(
                isinstance(x, (int, float))
                and not isinstance(x, bool)
                and math.isfinite(x)
                for x in current
            )
        ):
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

        moving = mc.is_moving()
        error = mc.get_error_information()

        if moving != 0 or error != 0:
            print(f"Movement blocked: moving={moving}, error={error}")
            print("Stop and check the arm before continuing.")
            break

        change = target - current_angle

        print(f"\nJ{joint}: {current_angle:.2f} -> {target:.2f}")
        print(f"Angle change: {change:+.2f} degrees")

        confirm = input("Type MOVE to execute (anything else cancels): ")

        if confirm.strip() != "MOVE":
            print("Movement cancelled.")
            continue

        mc.send_angle(joint, target, SPEED)
        print("Command sent. Verify motion has finished.")
        print("Check clearance before the next command.\n")

except KeyboardInterrupt:
    print("\nInterrupted. Requesting stop.")
    try:
        mc.stop()
    except Exception as exc:
        print(f"Software stop failed: {exc}")

print("Controller closed.")
