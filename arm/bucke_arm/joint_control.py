"""Interactive Pi controller and laptop simulator."""

import argparse

from .arm import FakeArm, RealArm


def _move_joint(arm, joint):
    """Show the current angle and proposed change before asking for MOVE."""
    # The menu and SDK number joints J1-J6. The telemetry tuple starts at zero.
    current = arm.get_angles()[joint - 1]
    low, high = arm.joint_limits(joint)
    if low >= high:
        raise ValueError(f"Invalid limits for J{joint}")

    print(f"J{joint} is at {current:.2f} degrees. Allowed command target: {low:g} to {high:g} degrees.")
    target = float(input("Desired angle: "))
    # The range check also rejects NaN and infinite input.
    if not low <= target <= high:
        raise ValueError("Target outside allowed range")

    print(f"J{joint}: {current:.2f} -> {target:.2f} ({target - current:+.2f} degrees)")
    if input("Type MOVE to execute: ").strip() != "MOVE":
        print("Movement cancelled.")
        return

    # The displayed limits are for the operator. The arm rechecks limits and
    # motion status after confirmation, immediately before sending the command.
    arm.move_joint(joint, target)
    print("Command sent. Verify motion is complete.")


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

    # R/S request the saved all-joint transition without another prompt.
    # O/C send gripper commands. Both use the checks in the selected arm.
    actions = {
        "R": arm.move_to_ready,
        "S": arm.move_to_standby,
        "O": arm.open_gripper,
        "C": arm.close_gripper,
    }

    print("Buck-E arm" + (" (simulated)" if args.fake else ""))
    print("1-6: move joint | O/C: open/close gripper | R/S: ready/standby | Q: quit")
    print("R/S start the saved all-joint move immediately. Verify the route is clear.")
    try:
        while True:
            choice = input("Select: ").strip().upper()
            try:
                if choice in actions:
                    actions[choice]()
                    print("Pose confirmed." if choice in ("R", "S") else "Gripper command sent.")
                # Individual joints have a separate confirmation step.
                elif choice in ("1", "2", "3", "4", "5", "6"):
                    _move_joint(arm, int(choice))
                else:
                    # Q or any unrecognized selection exits the menu.
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
