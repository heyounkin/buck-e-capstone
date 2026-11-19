# Buck-E arm

**Pickup execution disabled:** after the reported low approach and late gripper
rotation, `pickup --execute` and `RealArm.pick_up_block` refuse before hardware
queries or movement. Earlier execution instructions below are historical.
Offline target previews remain available; do not rerun the Pi pickup code.


The arm runs on a myCobot 280 Pi with Python 3.8.10, pymycobot 4.0.6 and
pyserial 3.5. Laptop commands and tests use the Python standard library, with
Matplotlib needed for the optional workspace plot.

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

## Plot the workspace

From this folder on your laptop:

```sh
python -m pip install matplotlib
python -m bucke_arm.workspace
```

The plot samples 30,000 joint configurations within Buck-E's configured angle
ranges and uses forward kinematics to calculate the wrist flange position for
each one. The window shows a rotatable 3D scatter plot in base millimetres,
colored by height. Add `--units in` to display the axes and height color bar in
inches. Add `--samples 5000` for a quicker run or `--seed 13` for a
different random sample. Save a plot to PNG with `--output workspace.png`.

Add `--boundary-only` to keep the farthest sampled point in each angular sector
around the point cloud's center, plus the minimum and maximum along each axis.
For example, `python -m bucke_arm.workspace --boundary-only --units in` shows
the sampled outer envelope of the wrist flange in inches. The result
approximates the outer boundary and omits internal gaps.

Use `--point gripper` to plot the estimated closed-pad midpoint in mat
coordinates using the provisional tool calibration. Both plot modes run
offline. The point clouds do not account for collisions, table clearance, or
whether a continuous path exists to each sampled point.

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
standby. Select P to query the current angles and gripper XYZ. Entering a valid
individual joint target executes the movement. R/S execute immediately.
Keep the sweep clear and close other serial
clients before moving. Ctrl+C requests stop, and exiting closes the connection.

Select `V` to set the speed for individual joint commands. Enter an integer
from 1 to 100. The menu starts each session at 10 and shows the current
setting when executing a joint command. R/S keep speed 5, and the gripper uses its own speed.

After an arm movement, the menu prints estimated gripper midpoint X, Y and Z
in mat inches, alongside the six actual joint readings used in that estimate.
P takes a fresh read-only snapshot using the same calculation. A printed
snapshot can become stale if a joint moves afterward. Individual joint
commands wait up to 120 seconds for three
consecutive stationary, fault-free readings within 3 degrees of the expected
angles before reporting XYZ. The estimate uses the final joint readings and
one fixed flange-local pad offset that includes the camera mount and gripper,
followed by one base-to-mat transform with zero base roll and pitch. This
provisional fit uses the November 10 Ready/Standby measurements and predicts
those two pad positions within 2.3 mm. It uses the closed-pad reference even
when the jaws are open. Mounting changes require recalibration. Independent
accuracy elsewhere in the workspace remains unverified. See the
[workspace validation notes](docs/workspace_validation.md) for the measurements,
parameters and remaining flange residuals.

The menu displays the Buck-E range, the robot's configured range when
available, and their overlap as the allowed target range. The simulator uses
the Buck-E range. Buck-E caps J2 at ±135° and J4 at ±145°. The Pi and SDK
allow ±140° and ±150° respectively. See the
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
Malformed telemetry errors identify `get_angles()` or `get_coords()` and show
the raw reply. Both queries must return six finite numbers for status to succeed.
`pi_status.py` remains a standalone raw Pi diagnostic for firmware, joint angles,
motion/errors, servo status, voltages and temperatures.

The connection-free preview computes signed differences and optionally checks a
candidate waypoint route. It uses the same ready pose as both arm classes:

```sh
python -m bucke_arm move-to-ready --angles -90 0 0 0 90 -130 --moving 0 --error 0
python -m bucke_arm move-to-ready --angles -90 0 0 0 90 -130 --moving 0 --error 0 --route config/ready_route.example.json
```

The example route is synthetic, not an execution path. `--limits PATH.json`
selects different offline bounds. The bundled defaults use Buck-E's command
limits, including the J2 and J4 caps. Physical travel and clearance remain unverified.
The preview runs offline.

## Hover experiments

The hover tool is separate from the saved R/S transitions. The operator reported
gripper-to-arm contact during direct hover return on November 5, 2026. The
hardware return is locked out. The [hover guide](docs/resume_hover.md) covers the
default single-command forward motion without correction trims, updated FK
endpoint reporting, measured trajectory capture, and offline reverse preview.
`python3 -m bucke_arm.pickup` previews the grasp for the one-inch block at
(7, 2). With physical clearance confirmed, `--execute` performs one approach
from saved ready to a 5/8-inch pad-midpoint height, verifies arrival, then
commands jaw closure. Begin with jaws already open; see the hover guide for
setup. The arm remains at grasp height; the later lift/sweep/drop is deferred.

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

## One full cycle

Place the one-inch block at (7,2) before starting from saved READY.
`python3 -m bucke_arm.full_cycle` previews; `--fake` replays checkpoints without
hardware. `--execute` runs once without placement or return prompts, using the
same checked approach/grasp/lift/sweep/drop motions. Return J6 uses speed 10;
other arm stages use 5, with J5/J1 issued together at the end of the return.
`--place-at-pickup` retains the old manual insertion pause if needed.

The first approach with the block already present still needs physical
verification; the previously demonstrated approach was block-out. Watch for
unintended contact before grasping. No block detection, feeder or automatic
repeat is added. See the hover guide for checks and observed phase details.

## Return from drop

`python3 -m bucke_arm.return_ready` previews the checked return from the observed
empty-gripper drop pose. `--execute` advances without confirmations: raise J2,
set J6 to -135 at speed 10, reset J3/J4, then move J5/J1 together in one final
READY command at speed 5. Start, limit, fault and completion checks remain.
`--step` optionally pauses before stages for diagnostics. The earlier separate
return was physically confirmed; the paired final stage is a new software-tested
change. The old direct hover return stays disabled.
`pick_and_place --execute --return-ready` and `full_cycle --execute` use the same
no-prompt return. The full-cycle block-placement prompt is retained.

## Copy updates to the Pi

Run these commands in Windows PowerShell, replacing `PI_ADDRESS` with the Pi's
current address. Close other serial clients before running the tests on the Pi.

```powershell
Set-Location "C:\Users\heath\OneDrive\Documents\Repos\buck-e-capstone"
ssh er@PI_ADDRESS 'cd /home/er/buck-e-capstone && mkdir arm-code-backup-20261118 && mv arm/bucke_arm arm/tests arm-code-backup-20261118/'
scp -r arm er@PI_ADDRESS:/home/er/buck-e-capstone/
```

The SSH command preserves the old package and tests outside the active arm
folder, preventing retired files from being mixed into the new suite. The
backup name must be unused; choose a new suffix when repeating this procedure.
Proceed with the copy only if the backup command succeeds. Reports, recordings,
and local configuration remain in the existing arm folder. Do not run arm
commands while the transfer is in progress.

On the Pi, verify without motion:

```sh
cd ~/buck-e-capstone/arm
python3 -m unittest discover -s tests
python3 -m bucke_arm.pickup
```

The current suite has 115 tests. The preview must report the grasp midpoint at
(7, 2, 0.625) inches. These checks do not execute pickup.
