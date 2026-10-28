# Buck-E arm

The Pi uses myCobot 280 Pi, Python 3.8.10, pymycobot 4.0.6 and pyserial 3.5.

## Arm classes

`bucke_arm/arm.py` contains two implementations with matching application methods:

- `RealArm`: owns the SDK connection, telemetry and hardware commands.
- `FakeArm`: tracks angles and gripper state in memory for laptop development.

Both provide `get_angles()`, `read_status()`, `move_joint()`, `open_gripper()`,
`close_gripper()`, `move_to_ready()`, `move_to_standby()`, `stop()` and `close()`.
No separate read-only interface, adapter or factory is needed. Importing either
class does not open a serial connection; constructing `RealArm()` does.

```python
from bucke_arm import FakeArm, RealArm

arm = FakeArm()          # On the Pi, use RealArm() instead.
try:
    arm.move_to_ready()  # Requires the saved standby starting pose.
    print(arm.get_angles())
    arm.move_to_standby()
finally:
    arm.close()
```

`FakeArm` starts at standby unless given an initial six-angle pose. Commands
update its state immediately. Tool coordinates remain fixed sample data;
it does not model motion, geometry, collisions or positioning accuracy.

## Manual testing

`joint_control.py` is a small menu that calls the arm methods. On the Pi:

```sh
cd ~/buck-e-capstone/arm
python3 -m bucke_arm.joint_control
```

For the same menu on a laptop without the SDK or robot:

```sh
python -m bucke_arm.joint_control --fake
```

Select 1-6 for an individual joint, O/C for the gripper, R for ready, or S for
standby. Joint commands use `MOVE` confirmation; R/S use `READY`/`STANDBY`.
Ctrl+C requests stop and exiting closes the connection. Keep other programs
communicating with the robot closed. Input prompts live in the menu; reusable
arm methods do not prompt.

## Saved standby and ready poses

The classes and offline preview share the same ready-pose constant:

- Standby: `[0, 0, 90, 90, 90, -135]`.
- Ready: `[90, 0, 0, 0, 90, -135]`.

On October 28, 2026 the user confirmed both synchronous transitions work safely.
`RealArm.move_to_ready()` requires standby and `move_to_standby()` requires ready.
Already at the requested pose returns without sending another command.

The reusable class retains starting-state, target-limit and completion checks.
It sends `sync_send_angles()` at SDK speed setting 5 with a nominal 120-second
timeout, then requires three consecutive stationary samples within 3 degrees
per joint during a 5-second final check. The SDK call blocks application fault
polling and returns 1 even on timeout, so its return value alone cannot establish
arrival. Failed post-command checks and exceptions request stop without an
application retry or automatic return. This tolerance is not calibrated accuracy.
SDK command handling may itself retry transmission. Closing the connection
neither stops movement nor releases holding torque.

## Status and offline preview

```sh
python -m bucke_arm status                 # FakeArm; no SDK required
python -m bucke_arm status --backend real  # RealArm; query telemetry on the Pi
```

The older `--backend hardware` spelling is accepted as an alias for `real`.
Hardware status failures are reported without falling back to simulated data.
`pi_status.py` remains a standalone raw Pi diagnostic for firmware, joint angles,
motion/errors, servo status, voltages and temperatures.

The connection-free preview computes signed differences and optionally checks a
candidate waypoint route. It uses the same ready pose as both arm classes:

```sh
python -m bucke_arm move-to-ready --angles 90 0 0 0 90 -130 --moving 0 --error 0
python -m bucke_arm move-to-ready --angles 90 0 0 0 90 -130 --moving 0 --error 0 --route config/ready_route.example.json
```

The example route is synthetic, not an execution path. `--limits PATH.json`
selects different offline bounds. The bundled bounds are provisional, and
neither previews nor `FakeArm` validate physical clearance. The preview never
opens a hardware connection or executes movement.

## Tests and history

From this folder:

```sh
python -m unittest discover -s tests -v
```

Tests cover both classes, synchronous saved-pose transitions, status, the manual
menu and offline planning. All use software fixtures and do not move the robot.
Old sequential, J4 recovery, J5 stepping and resting-log experiments are removed.
Their evidence remains in `docs/pi_verification.md` and `docs/standby_plan.md`;
commands in those historical notes may name retired scripts or architecture.

## Copy updates to the Pi

From Windows PowerShell, replace `PI_ADDRESS`:

```powershell
scp -r "C:\Users\heath\OneDrive\Documents\Repos\buck-e-capstone\arm" er@PI_ADDRESS:/home/er/buck-e-capstone/
```

This overwrites matching files without deleting retired files already on the Pi.
