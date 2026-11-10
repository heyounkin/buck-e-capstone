"""Interactive Pi controller and laptop simulator."""

import argparse

from .arm import (
    DEFAULT_JOINT_SPEED, FakeArm, MOVE_TIMEOUT, POSE_SPEED, READY_ANGLES, RealArm,
    SPEC_LIMITS, STANDBY_ANGLES, validate_speed,
)


def _move_joint(arm, joint, speed):
    """Show the current angle and proposed change before asking for MOVE."""
    # The menu and SDK number joints J1-J6. The telemetry tuple starts at zero.
    current = arm.get_angles()[joint - 1]
    local_low, local_high = SPEC_LIMITS[joint - 1]
    (low, high), controller_range = arm.joint_limit_ranges(joint)

    print(f"\n--- JOINT {joint} ---")
    print(f"Current angle: {current:.2f} degrees")
    print(f"Buck-E range: {local_low:g} to {local_high:g} degrees")
    if controller_range is not None:
        robot_low, robot_high = controller_range
        print(f"Robot configured range: {robot_low:.1f} to {robot_high:.1f} degrees")
    else:
        print("Robot limits unavailable; using Buck-E limits.")

    if low >= high:
        raise ValueError(f"Invalid limits for J{joint}")

    print(f"Allowed target range: {low:.1f} to {high:.1f} degrees")
    target = float(input("Desired angle: "))
    # The range check also rejects NaN and infinite input.
    if not low <= target <= high:
        raise ValueError("Target outside allowed range")

    print(f"\nJ{joint}: {current:.2f} -> {target:.2f}")
    print(f"Angle change: {target - current:+.2f} degrees")
    print(f"Joint speed setting: {speed}")
    if input("Type MOVE to execute: ").strip() != "MOVE":
        print("Movement cancelled.")
        return

    # The displayed limits are for the operator. The arm rechecks limits and
    # motion status after confirmation, immediately before sending the command.
    arm.move_joint(joint, target, speed=speed)
    print("Command sent. Verify motion is complete.\n")


def _move_to_saved_pose(arm, choice):
    """Show the selected transition and delegate the movement checks to the arm."""
    if choice == "R":
        name, start_name = "READY", "STANDBY"
        start, target, move = STANDBY_ANGLES, READY_ANGLES, arm.move_to_ready
    else:
        name, start_name = "STANDBY", "READY"
        start, target, move = READY_ANGLES, STANDBY_ANGLES, arm.move_to_standby

    print(f"\n--- {name} TRANSITION ---")
    print(f"Required {start_name.lower()}:", list(start))
    print(f"Target {name.lower()} pose:", list(target))
    print("This moves multiple joints. Verify the entire route is clear.")
    move()
    print(f"{name} POSITION CONFIRMED.")


def main(argv=None):
    """Open the selected arm and run the interactive controller."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fake", action="store_true", help="Run without the Pi SDK or serial connection")
    args = parser.parse_args(argv)

    # Importing this module leaves the serial port closed. Only the real
    # backend opens it, after command-line arguments have been parsed.
    try:
        arm = FakeArm() if args.fake else RealArm()
    except (RuntimeError, OSError) as exc:
        print(f"Unable to open controller: {exc}")
        return 1

    # This setting lasts for the menu session and applies to individual joints.
    joint_speed = DEFAULT_JOINT_SPEED

    print("\n=== BUCK-E WORKSPACE CONTROLLER ===")
    if args.fake:
        print("(simulated)")
    print("1-6 : Move individual joint")
    print("O   : Open gripper")
    print("C   : Close gripper")
    print("R   : Move from standby to ready")
    print("S   : Move from ready to standby")
    print(f"V   : Set individual joint speed (default {joint_speed})")
    print("Anything else : Quit\n")
    print(f"R/S: execute synchronous all-joint movement immediately, speed {POSE_SPEED}.")
    if not args.fake:
        print(f"SDK timeout: {MOVE_TIMEOUT} seconds.")
    try:
        while True:
            choice = input("Select (1-6 / O / C / R / S / V): ").strip().upper()
            try:
                # R/S request the saved all-joint transition without another prompt.
                if choice in ("R", "S"):
                    _move_to_saved_pose(arm, choice)
                # Gripper commands use the checks in the selected arm.
                elif choice in ("O", "C"):
                    if choice == "O":
                        print("Opening gripper...")
                        arm.open_gripper()
                    else:
                        print("Closing gripper...")
                        arm.close_gripper()
                    print("Gripper command sent.\n")
                elif choice == "V":
                    try:
                        speed = int(input(f"Joint speed (1-100, current {joint_speed}): "))
                    except ValueError:
                        raise ValueError("Enter a whole number for joint speed") from None
                    validate_speed(speed)
                    joint_speed = speed
                    print(f"Joint speed set to {joint_speed}.")
                # Individual joints have a separate confirmation step.
                elif choice in ("1", "2", "3", "4", "5", "6"):
                    _move_joint(arm, int(choice), joint_speed)
                else:
                    # Q or any unrecognized selection exits the menu.
                    print("Exiting controller.")
                    break
            except (ValueError, RuntimeError) as exc:
                print(f"Command failed: {exc}")
    except EOFError:
        # Closed input exits through the same cleanup as Q.
        pass
    except KeyboardInterrupt:
        print("\nInterrupted. Requesting stop.")
        try:
            # On hardware, stop requests both arm and gripper motion to stop.
            arm.stop()
        except Exception as exc:
            print(f"Software stop failed: {exc}")
    except OSError as exc:
        print(f"Controller communication failed: {exc}")
        return 1
    finally:
        # Release the serial connection even if a command or stop fails.
        arm.close()

    print("Controller closed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
