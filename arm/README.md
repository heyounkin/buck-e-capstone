# Buck-E arm

The arm runs on a myCobot 280 Pi with Python 3.8.10, pymycobot 4.0.6 and
pyserial 3.5. Laptop commands and tests use the Python standard library.

## Arm classes

`bucke_arm/arm.py` contains two implementations with matching application methods:

- `RealArm`: owns the SDK connection, telemetry and hardware commands.
- `FakeArm`: tracks angles and gripper state in memory for laptop development.

Both provide `get_angles()`, `read_status()`, `move_joint()`, `open_gripper()`,
`close_gripper()`, `move_to_ready()`, `move_to_standby()`, `stop()` and `close()`.
Use `joint_limits(joint)` to display the allowable angle range before a move.
Constructing `RealArm()` opens the serial port. Imports and `FakeArm()` work offline.

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
update its state immediately. Flange coordinates follow a nominal forward-
kinematics model; it does not model motion physics, collisions or positioning
accuracy. Model RPY conventions have not yet been matched to SDK orientation.

## Offline forward kinematics

From this folder:

```sh
python -m bucke_arm fk --angles 108.01 -19.59 11.07 145.81 76.28 -16.25 --reported-xyz 20.5 -1.3 290.5
```

This computes nominal base-to-flange XYZ, orientation and a 4x4 transform without
connecting to the arm. The optional supplied XYZ produces a position-error
comparison. See [model conventions and historical comparisons](docs/forward_kinematics.md)
for geometry, frame assumptions, Python usage and remaining physical validation.

To collect fresh read-only comparisons on the Pi at an existing stationary pose:

```sh
python3 -m bucke_arm.fk_capture --pose standby
python3 -m bucke_arm.fk_capture --pose ready
```

Run each command only at its named pose, closing the manual controller first.
Use the existing R/S controller between captures. Each run saves three paired
samples and XYZ comparisons under `fk_reports/`. See the linked FK guide for
transfer instructions and sampling checks. Capture itself never moves the arm.

## Manual testing

The manual menu uses the same arm methods as application code. On the Pi:

```sh
cd ~/buck-e-capstone/arm
python3 -m bucke_arm.joint_control
```

For the same menu on a laptop without the SDK or robot:

```sh
python -m bucke_arm.joint_control --fake
```

Select 1-6 for an individual joint, O/C for the gripper, R for ready, or S for
standby. Joint commands use `MOVE` confirmation; R/S execute immediately without
a second confirmation prompt. Keep the sweep clear and close other serial
clients before moving. Ctrl+C requests stop, and exiting closes the connection.

The menu displays the range of allowed command targets in degrees. On the Pi
that range is the overlap of the pinned SDK limits and valid live controller
limits. The simulator uses the SDK limits. On November 9, 2026 the Pi reported
the same limits as the SDK for all six joints, including J2 and J4 at ±140°
and ±150° respectively, and J5 at −155° to +160°. See the
[controller readback](docs/pi_verification.md#joint-limit-readback-november-9-2026).

## Saved standby and ready poses

The classes and offline preview share the same ready-pose constant:

- Standby: `[0, 0, 90, 90, 90, -135]`.
- Ready: `[-90, 0, 0, 0, 90, -135]`.

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
python -m bucke_arm move-to-ready --angles -90 0 0 0 90 -130 --moving 0 --error 0
python -m bucke_arm move-to-ready --angles -90 0 0 0 90 -130 --moving 0 --error 0 --route config/ready_route.example.json
```

The example route is synthetic, not an execution path. `--limits PATH.json`
selects different offline bounds. The bundled defaults match the SDK and the
November 9 controller readback. Physical travel and clearance remain unverified.
The preview runs offline.

## Hover experiments

The hover tool is separate from the saved R/S transitions. The operator reported
gripper-to-arm contact during direct hover return on November 5, 2026. The
hardware return is locked out. The [hover guide](docs/resume_hover.md) covers the
forward route, measured trajectory capture, and offline reverse preview.

## Tests and project records

From this folder:

```sh
python -m unittest discover -s tests -v
```

Tests use simulated arms and SDK fixtures. Generated capture files in
`fk_reports/` and `hover_trajectories/` stay local.

- [Pi verification record](docs/pi_verification.md) for hardware observations and motion history.
- [Workspace validation](docs/workspace_validation.md) for frame and clearance estimates.
- [Standby planning notes](docs/standby_plan.md) for early experiments and their evidence.

## Copy updates to the Pi

From the repository root, replace `PI_ADDRESS` with the Pi hostname or address:

```powershell
scp -r arm er@PI_ADDRESS:/home/er/buck-e-capstone/
```

This overwrites matching files without deleting retired files already on the Pi.
