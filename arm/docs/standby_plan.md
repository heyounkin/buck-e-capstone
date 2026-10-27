# Standby pose: preliminary offline review

> Cleanup, October 21, 2026: the old J1/J5 test helpers, J1 positioning
> helper, and fake-serial packet-check script have been removed. References
> to those scripts below are historical; use the current arm README for retained tools.

## User-specified safety ready position

On October 21, 2026, the user specified the safety ready position in J1-J6
order, in degrees, corrected to: `[-90, 0, 0, 0, 90, -135]`.
This is the selected ready target; the upright candidate discussed below is
historical preliminary analysis. The supplied angles do not establish a tested
route from the current pose, endpoint accuracy, or physical clearance. Recording
this target does not execute motion or enable automatic initialization.

**Correction:** after the user-run guided phase stopped at checkpoint 1 on the
assistant-selected 49 C gate, the user explicitly requested completing the
load-reduction task without that gate. The active script now validates and
reports temperatures without an invented numeric cutoff. Actual controller/
servo fault checks, motion checks and local confirmation remain. No motor
command was shown before that preflight refusal. All 94 offline tests pass.
References to the 49 C gate below describe superseded preparation.

**Latest observation:** the combined stage produced J5 72.24 -> 68.37 degrees,
with unchanged other-joint endpoint readings, moving 0/error 0, and J5 48 C
before/after. Target 67.33 was missed by +1.04 degrees. The user locally typed
`MOVE J5` after the displayed prompt about prior physical isolation and clearance.
This does not independently confirm physical isolation of the newly completed
move; that remains a prerequisite to accepting the next local prompt.

**Next prepared phase:** the same `pi_standby_step.py --hardware --toward-zero`
offers the fixed J5-only targets documented in the arm README, from reviewed
68.37 degrees down to zero. It pauses for a new explicit local authorization
before every step. Faults, unexpected movement, hot readings, cancellation, or
endpoint error above the experimental 1.25-degree progression gate cancel the
remaining targets. Other-joint references are not rebased across the phase.
The +/-0.25-degree accuracy result continues to be reported separately, including
failure. All 92 offline tests passed; the guided hardware phase is pending.
Each next wrist/plate/cable sweep needs the operator's physical check. Neither
the initial model nor supplied still photos certify the entire arc. Completion
would establish only this wrist phase, not a full ready/standby or thermal pass.

The following single-step descriptions are retained as historical preparation.

**Current status:** the user independently ran the first J5 target using a short
Pi script. The immediate endpoint was `[108.10, -19.59, 11.07, 146.42, 72.33, -16.78]`.
The earlier helper below is now historical and must not be rerun. The next
prepared combined stage is described in the arm README and implemented in
`pi_standby_step.py`: confirm the settled starting state and physical isolation
locally, then offer a single fixed J5 target 67.33 degrees at speed 10. No
separate chat approval is requested; `MOVE J5` authorizes the displayed stage
only after the operator's physical checks. All 84 offline tests passed; hardware
execution is pending. This is not an automatic route to the upright candidate.

The immediate goal is a reduced-load waiting pose with the gripper detached.
Picking above mat (7,2) is a later milestone. No motion is authorized or
implemented by this document. Candidate angles below must not be sent as an
all-joint command. A current physical clearance review and supervised tests are
still required.

## Evidence and model

Latest reported joint angles, in degrees:
`[108.01, -19.59, 11.07, 145.81, 76.28, -16.25]`.
Controller flange XYZ: `[20.5, -1.3, 290.5]` mm. J4 recovery and three stationary,
error-free readings were observed before the latest coordinate query. The user
confirmed J4 holds without support. Latest reported J5 temperature: 49 C;
thermal limits for the exact hardware remain unverified.

The manufacturer's [myCobot 280 Pi URDF](https://github.com/elephantrobotics/mycobot_ros/blob/noetic/mycobot_description/urdf/mycobot_280_pi/mycobot_280_pi.urdf)
was retrieved on October 21, 2026. Forward kinematics were calculated offline
using each joint origin's Rz(yaw) Ry(pitch) Rx(roll), followed by rotation about
the documented local joint Z axis. SDK joint degrees were used in chain order.
The current-pose prediction is `[20.34, -0.06, 291.07]` mm, about 1.38 mm from
the controller result. This single comparison supports preliminary use only;
it does not verify the exact hardware geometry, frame origin, or calibration.

The URDF contains no inertial elements. No link masses, centers of mass, motor
currents, actual motor torques, temperatures, or collision meshes were evaluated
in these calculations. The supplied nominal effort values are not validated
motor ratings. A low-load thermal outcome cannot be certified from this model.

## Candidate and first movement direction

An upright candidate retains J1 and J6 while bringing J2-J5 toward zero:
`[108.01, 0, 0, 0, 0, -16.25]` degrees. Its modeled J5 axis is vertical, so
gravity has no component of moment about that ideal axis. Actual heating can
still occur. This is an engineering candidate, not a manufacturer-certified
standby pose or a selected collision-free route. It is tall (modeled flange
height 418.14 mm), so overhead clearance must be reviewed.

First candidate test direction: decrease J5 from 76.28 toward 71.28 degrees,
retaining the other joints. This moves toward a wrist configuration with less
modeled gravitational moment for a point mass at the flange. It is more
directly connected to the wrist-load question than the earlier J2-only test.

| Hypothetical J5 | Modeled flange XYZ (mm) | Gravity moment arm about J5 for a point mass at flange (mm) |
| --- | --- | --- |
| 76.28 degrees, baseline | 20.34, -0.06, 291.07 | 30.05 |
| 71.28 degrees | 22.26, -2.39, 293.66 | 29.29 |
| 0 degrees, wrist-only endpoint candidate | 70.37, -10.67, 314.67 | approximately 0 |

The first five degrees predict about 2.6 mm of flange rise. The lever calculation
is only `abs(axis dot ((flange - J5 pivot) cross vertical))`; it is not actual
wrist torque and does not account for the wrist's distributed mass, cables,
friction, or electronics. No claim that this first step will cool J5 is made.
Do not infer clearance of the entire wrist from flange displacement.

## Before implementation or execution

1. Review a current side view after gripper removal and J4 recovery. Earlier
   pictures and earlier helper baselines do not establish the present sweep.
2. Review the first fixed J5 test, speed, expected packet, supervision, and
   abort conditions. Obtain authorization for that specific new joint test.
3. Prepare and verify a preview-first helper before execution. It must not
   compute a fresh target from an unreviewed starting pose or automatically
   continue toward zero.
4. Verify each physical result before planning subsequent segments. In
   particular, a J4 sweep from near 146 degrees toward zero can pass through
   configurations with more wrist loading; no such sweep is approved here.
5. Validate the eventual standby by monitored holding and temperature trend
   against applicable manufacturer guidance. Do not promise a no-heating pose.

Keep the arm powered down and physically supported during offline planning.
No additional powered coordinate queries are needed for the present model review.

## J5 helper prepared after current photo review

The user supplied three current side views with the gripper detached. J4 holding
without support was explicitly confirmed earlier. The wrist and mounting plate
are visible, with a loose black cable close to the wrist/upper link. These views
do not measure internal clearance or certify the entire sweep. The operator must
verify the wrist gap, cable slack, secure mounting, and reachable switch before
execution. No additional photographs are requested for this same starting pose.

`bucke_arm.test_joint5` is prepared, with hardware authorization and execution
pending. It offers only fixed target J5 71.28 degrees at speed setting 10 from
reviewed pose `[108.01, -19.59, 11.07, 145.81, 76.28, -16.25]`. Starting tolerance
is 0.2 degrees for J5 and 0.5 for other joints; three samples must be stable
within 0.2 degrees. Firmware, SDK, serial constants, free mode, enabled servos,
diagnostics, stationary/error-free status, and servo health are checked. State
is rechecked after the `MOVE J5` prompt.

The async movement trace permits only packet `fe fe 06 21 05 1b d8 0a fa` and a
complete nine-byte host write, then captures a raw reply. No retry, return,
correction, next joint, reference-setting, calibration, or power/torque change
is sent. Monitoring lasts at most a nominal ten seconds, checked between
synchronous queries; communication delays may extend wall time. Other-joint
drift over 0.5 degrees, J5 outside target/start interval with 0.25-degree margin,
invalid readings, faults, interruption, failed write, or unmet target requests
stop. Three stationary readings within the unchanged +/-0.25-degree tolerance
after at least two seconds give a telemetry pass. Physical isolation must still
be confirmed. Expected shortfall is not silently reclassified as success.

The helper refuses or stops if J5 exceeds 49 C, the last reviewed reading. This
is an experimental conservative gate, not a manufacturer thermal rating, proof
of thermal safety, or temperature control. Stop preserves holding torque and
does not guarantee cooling. Do not leave the arm powered while waiting for
review. All 75 offline tests passed; actual SDK encoding and physical execution
of the new helper have not been tested on the Pi.

Transfer from Windows PowerShell (no robot movement):

```powershell
scp "C:\Users\heath\OneDrive\Documents\Repos\buck-e-capstone\arm\bucke_arm\test_joint5.py" "C:\Users\heath\OneDrive\Documents\Repos\buck-e-capstone\arm\bucke_arm\test_joint1.py" er@172.30.2.214:/home/er/buck-e-capstone/arm/bucke_arm/
```

Preview on the Pi (no connection):

```sh
cd ~/buck-e-capstone/arm
python3 -m bucke_arm.test_joint5
```

Hardware invocation after authorization and operator clearance checks:
`python3 -u -m bucke_arm.test_joint5 --hardware`. Confirm only this displayed
single J5 plan with `MOVE J5`; do not use the prior J1 helpers. If the starting
pose differs, share the output without changing constants or repositioning.
