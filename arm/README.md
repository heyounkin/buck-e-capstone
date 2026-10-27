# Buck-E arm

The arm is a myCobot 280 Pi with Python 3.8.10, pymycobot 4.0.6, and pyserial 3.5.
Keep other programs that communicate with the robot closed while running a script.

## Start with the simple Pi script

Read [pi_status.py](pi_status.py) first. It uses the robot library directly:

```python
from pymycobot import MyCobot280, PI_PORT, PI_BAUD

arm = MyCobot280(PI_PORT, PI_BAUD)
```

`PI_PORT` is `/dev/ttyAMA0`; `PI_BAUD` is `1000000`. These come from the installed
library and match the settings tested on this Pi. The script checks the library
version and connection settings before connecting.

On the Pi, run:

```sh
cd ~/buck-e-capstone/arm
python3 pi_status.py
```

The script prints firmware, three samples of joint angles/motion/error status,
and servo status, voltages, and temperatures. Each angle list is ordered
`J1, J2, J3, J4, J5, J6`, in degrees. It prints raw responses so unexpected
values remain visible. A query failure prints a normal Python traceback so you
can see the failing line. It does not replace missing readings with simulated data.

The `try` block contains the queries. The `finally` block closes the serial
connection even if a query fails. Closing communication does not stop a moving
robot or alter holding torque. This script only reads; it does not send motion,
release, calibration, power, or error-clear commands.

## Copy the updated files from Windows

In Windows PowerShell, replace `PI_ADDRESS` with the SSH hostname or IP:

```powershell
scp -r "C:\Users\heath\OneDrive\Documents\Repos\buck-e-capstone\arm" er@PI_ADDRESS:/home/er/buck-e-capstone/
```

This also replaces the Pi movement helper with the current supervised version.
Matching destination files are overwritten; unrelated files are not deleted.

## Packaged status and laptop simulation

The capstone package still provides validated JSON status. From this folder:

```sh
python3 -m bucke_arm status                     # simulated status; no robot
python3 -m bucke_arm status --backend hardware  # actual read-only telemetry
python3 -m unittest discover -s tests -v        # offline tests; no robot
```

On Windows, use `python` instead of `python3`. Simulation and tests require no
third-party packages. The hardware adapter imports pymycobot only when selected
and uses the same Pi constants. Its telemetry checks reject malformed readings.

## Offline move_to_ready preview

The first `move_to_ready()` implementation is in `bucke_arm/ready.py`. It uses
the corrected target `[-90, 0, 0, 0, 90, -135]` degrees in J1-J6 order and
returns each joint's current angle, target, and signed difference. It does not
wrap differences modulo 360, automatically choose a route, or execute movement.
It checks configured limits and can preview an explicitly supplied waypoint route.

From the arm folder on Windows, try this **hypothetical stationary, error-free
input**, not a fresh reading or evidence that the outstanding error was resolved:

```powershell
python -m bucke_arm move-to-ready --angles -90 0.7 -0.52 -0.79 89.12 -134.12 --moving 0 --error 0
```

Use `python3` on the Pi. No SDK or robot is required. Both `--moving` and
`--error` must be supplied explicitly; moving or nonzero-error inputs are
refused with exit code 1. Malformed angles are rejected. This example returns
changes of approximately `[0, -0.7, 0.52, 0.79, 0.88, -0.88]` degrees.
Supplying `--error 5` reproduces refusal for the last reported fault. Use the
actual error code when describing physical telemetry.

The same calculation is available from Python:

```python
from bucke_arm.ready import move_to_ready

preview = move_to_ready(
    [-90, 0.7, -0.52, -0.79, 89.12, -134.12],
    moving=0,
    error_information=0,
)
```

Output always reports `mode: preview_only`, `hardware_execution_enabled: false`,
and `route_reviewed: false`. With the default provisional bounds,
`joint_limits_verified` is also false. It cannot verify
the supplied state. `exactly_at_target` compares the supplied numbers exactly;
it is not an application tolerance or a physical positioning pass. Joint changes
are listed by joint number, not as an execution order. Without `--route`,
`waypoint_steps` is empty and no route is proposed.

### Configurable joint limits

By default the preview loads [ready_limits.provisional.json](config/ready_limits.provisional.json).
Its J1-J6 minimums are `[-168, -135, -150, -145, -155, -180]` and maximums are
`[168, 135, 150, 145, 160, 180]` degrees. These come from the
[SDK 4.0.6 API table](https://github.com/elephantrobotics/pymycobot/blob/v4.0.6/docs/MyCobot_280_en.md#send_angleid-degree-speed).
The [product page](https://docs.elephantrobotics.com/docs/mycobot-pi-en/2-serialproduct/2.1-280/2.1.2.1%20Introduction%20of%20product%20parameters.html)
lists a different J5 range. The defaults are an offline development choice, not
verified limits for this controller or physical clearance boundaries.

To use a different limits file, add `--limits PATH.json`. Its exact fields are
`minimum_deg` and `maximum_deg` (six numbers each), `source` (nonempty text),
and `verified` (boolean). A true verification flag is only the configuration
author's declaration; loading it performs no hardware verification and enables
no movement. Preserve actual measurement provenance in `source` before marking
a future controller-derived configuration verified.

The current pose, ready target, every waypoint, and both bounds of a route's
starting range must fit the selected limits. Boundaries are inclusive. Nothing
is clamped or wrapped. Invalid configurations or out-of-bounds angles refuse
the entire preview. Missing files and malformed JSON also fail with exit code 1.

### Candidate waypoint routes and supported starts

[ready_route.example.json](config/ready_route.example.json) is a **synthetic
offline example**, not a physically reviewed route. It supports starts in the
inclusive six-joint box from `[-91, -1, -1, -1, 89, -131]` to
`[-89, 1, 1, 1, 91, -129]` degrees. Try:

```powershell
python -m bucke_arm move-to-ready --angles -90 0 0 0 90 -130 --moving 0 --error 0 --route config/ready_route.example.json
```

This returns two hypothetical segments: first to `[-90, 0, 0, 0, 90, -132]`,
then to the ready target at J6 -135. Each segment contains its own start,
endpoint, and signed joint differences. The final ready target is appended
automatically; consecutive duplicate poses are skipped. An input exactly at
ready produces no movement segments after configuration and start checks pass.

A route file has exactly these fields: `name`, `description`, `start_min_deg`,
`start_max_deg`, and `waypoints_deg`. The start bounds contain six numbers each;
waypoints are an ordered list of six-number poses, which may be empty. Each joint
must fit its supported start range; otherwise the preview refuses without
falling back to a direct move or another route. The entire route is checked
before output, including later waypoints. Unknown JSON fields are rejected.

From Python, pass `limits=load_limits(path)` and `route=load_route(path)` to
`move_to_ready()`, importing the loaders from `bucke_arm.ready_config`.
Limit checks cover numeric endpoints, not obstacle avoidance, tracking error,
or the controller's interpolation between them. No collision model is used.

Next implementation work: obtain controller limits and review supported starting
poses and physical routes, then select speed and endpoint tolerance before adding
supervised execution. No hardware backend is exposed by this command.
The earlier J5 standby helper is separate and does not implement this target.

## Optional read-only resting check

Run `python3 pi_rest_check.py` on the Pi to record 30 resting telemetry samples
and save a timestamped CSV. It sends no motion or setting changes. Keep other
controller programs closed. Its offline checks remain in `tests/test_pi_rest_check.py`.

## Standby-pose planning and proposed J5 step

**Temperature-gate correction:** the active `pi_standby_step.py` no longer uses
the assistant-selected 49 C cutoff. It was not a verified manufacturer limit
and blocked the intended unloading sequence at checkpoint 1 before movement.
Temperatures are still validated and reported, and controller/servo faults,
invalid telemetry, unexpected motion, and drift still stop execution. This
supersedes references to the 49 C gate in the historical preparation below.
Use the updated script with
`--hardware --toward-zero`; the target sequence and local prompts are unchanged.

**Latest result and current invocation:** the combined single step completed
with before J5 72.24, after J5 68.37 degrees, other reported joints unchanged,
moving 0/error 0, and J5 temperature 48 C before/after. The commanded target
67.33 was missed by +1.04 degrees; this is settled-motion evidence, not a
+/-0.25-degree accuracy pass. The old default single-step baseline is obsolete.

The updated same script offers a locally supervised J5-only phase:
`python3 -u pi_standby_step.py --hardware --toward-zero`. Its initial reviewed pose
is `[108.10, -19.59, 11.07, 146.42, 68.37, -16.78]`. Fixed targets at speed 10 are
63.37, 58.37, 53.37, 48.37, 43.37, 38.37, 33.37, 28.37, 23.37, 18.37, 13.37,
8.37, 3.37, and 0 degrees. **Every movement requires a new local `MOVE J5`** after
inspection of the wrist gap, plate, cables, and the prior physical movement.
Cancel any prompt to end the phase. There is no unprompted next movement.

The existing fault, enable, pose, temperature and drift checks remain. Other
joint references stay fixed across the phase to reject cumulative drift. J5's
next starting reference uses the observed settled endpoint; commanded targets
are fixed and never computed from live readings. A command is rejected if its
decrease would exceed 6.6 degrees from the observed baseline. An experimental
1.25-degree endpoint gate cancels remaining targets after a larger shortfall.
This progression gate reflects the observed approximately one-degree shortfall;
it is not the unchanged +/-0.25-degree accuracy criterion or calibrated accuracy.
The initial and intermediate paths are still subject to physical inspection;
model predictions and software checks do not establish collision clearance.

Default and `--toward-zero` without `--hardware` are previews. The final zero
target may leave a shortfall; there is no correction or repetition. Completing
this phase is not completion of the entire standby pose or thermal validation.
Guided phase hardware execution is pending. With the ready route preview, all 74
offline tests pass.

## Safety ready position

On October 21, 2026, the user corrected the ready target to
`[-90, 0, 0, 0, 90, -135]` degrees
in J1-J6 order. The target is recorded only; its route and application tolerance
remain unverified, and automatic initialization is not enabled.

The retained standby helper implements the earlier J5-only phase toward zero,
not a route to this new six-joint ready target. See
[standby_plan.md](docs/standby_plan.md) for the planning history and
[pi_verification.md](docs/pi_verification.md) for completed hardware observations.

## Removed experiments

The completed J1 motion and positioning experiments, superseded standalone J5
experiment, fake-serial packet check, and their dedicated unit tests were removed
during cleanup. Their historical results remain in `docs/pi_verification.md` and
`docs/standby_plan.md`; commands naming those removed scripts are historical.
The standby script now contains its own six-value telemetry validator and its
tests have an independent mock client.

The J4 recovery helper and its tests remain available as a record of the
supervised recovery procedure, not a routine initialization step.

## Where to look

- `pi_status.py`: short, direct pymycobot diagnostic script.
- `pi_rest_check.py`: read-only resting telemetry and CSV log.
- `pi_standby_step.py`: supervised J5 phase with local checkpoints.
- `bucke_arm/recover_joint4.py`: supervised manual recovery helper.
- `bucke_arm/hardware.py`: read-only adapter used by packaged status.
- `bucke_arm/arm.py`: telemetry validation and simulated arm.
- `bucke_arm/__main__.py`: command-line selection of simulation or hardware.
- `bucke_arm/ready.py`: corrected ready target and offline joint-change preview.
- `bucke_arm/ready_config.py`: joint-limit and candidate-route configuration validation.
- `config/ready_limits.provisional.json`: provisional offline joint limits.
- `config/ready_route.example.json`: synthetic waypoint and starting-range example.
- `docs/pi_verification.md`: observed hardware results and unresolved issues.
- `config/hardware.observed.json`: historical observations, not motion settings.
- `tests/`: offline checks using simulated SDK objects.

The constant definitions are in the manufacturer's
[pymycobot 4.0.6 source](https://github.com/elephantrobotics/pymycobot/blob/v4.0.6/pymycobot/__init__.py).
