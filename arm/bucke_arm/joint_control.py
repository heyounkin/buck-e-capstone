
from pymycobot import MyCobot280
import math

# myCobot 280 Pi connection
mc = MyCobot280("/dev/ttyAMA0", 1000000)

SPEED = 10
MAX_STEP = 15  # Maximum degrees per command

# Published myCobot 280 joint limits
LIMITS = {
    1: (-168, 168),
    2: (-135, 135),
    3: (-150, 150),
    4: (-145, 145),
    5: (-155, 160),
    6: (-180, 180)
}

print("=== Buck-E Joint Controller ===")
print("Joint 1-6 to move, anything else to quit.")

try:
    while True:
        choice = input("\nJoint (1-6): ").strip()

        if choice not in ("1", "2", "3", "4", "5", "6"):
            print("Exiting joint controller.")
            break

        joint = int(choice)

        try:
            angle = float(input("Desired angle (degrees): "))
        except ValueError:
            print("Invalid angle. Try again.")
            continue

        if not math.isfinite(angle):
            print("Angle must be finite.")
            continue

        low, high = LIMITS[joint]

        if not low <= angle <= high:
            print(f"Outside joint limits: {low} to {high}")
            continue

        # Do not command a moving or faulted arm
        if mc.is_moving() != 0:
            print("Arm moving or status unknown. Try again.")
            continue

        if mc.get_error_information() != 0:
            print("Controller error or unknown status. Stop testing.")
            break

        current = mc.get_angles()

        if not isinstance(current, (list, tuple)) or len(current) != 6:
            print("Could not read current angles. Stopping.")
            break

        change = angle - current[joint - 1]

        if abs(change) > MAX_STEP:
            print(f"Change too large ({change:.1f} degrees).")
            print(f"Use steps of at most {MAX_STEP} degrees.")
            continue

        print(f"Moving J{joint}: {current[joint-1]:.1f} -> {angle:.1f}")
        mc.send_angle(joint, angle, SPEED)
        print("Command sent. Check motion before continuing.")

except KeyboardInterrupt:
    print("\nInterrupted. Requesting arm stop.")
    mc.stop()

print("Controller finished.")
