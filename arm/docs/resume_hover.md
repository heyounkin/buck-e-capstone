# Continuous approach and pickup preparation

**Current full-cycle setup:** put the block at (7,2) before starting. The default
full cycle has no placement pause or return confirmations. `--place-at-pickup`
retains the previous manual insertion checkpoint when needed. The first
approach with the block already present still needs physical verification.


**Current return behavior:** return confirmations are removed by operator request.
J2, J6, J3 and J4 remain checked stages; the final all-joint command sends J5
and J1 together to READY. Return J6 uses 10; other stages use 5. Optional
`return_ready --step --execute` restores diagnostic pauses only when requested.
The block-placement prompt before pickup remains part of the full-cycle setup.


**Pickup execution disabled:** after the reported low approach and late gripper
rotation, `pickup --execute` and `RealArm.pick_up_block` refuse before hardware
queries or movement. Earlier execution instructions below are historical.
Offline target previews remain available; do not rerun the Pi pickup code.


## Current behavior (November 18)

HOVER now defaults to **one all-joint endpoint command**, speed 5, with measured
trajectory capture and three stationary samples within 1.5 degrees per joint.
It sends no intermediate lift/shift/lower waypoints or final correction trims.
Fault, starting-state, limit, completion and stop checks remain active. Monitoring
reads the motion; it does not add movement commands. The SDK determines joint
progress, so one command does not imply a straight Cartesian path.

From the Pi repository's `arm` folder:

```sh
python3 -m bucke_arm.resume_hover
python3 -m bucke_arm.resume_hover --execute
```

The first command previews without connecting. The second retains the local
HOVER confirmation and the setup/port requirements below. `--continuous` is an
alias for the default. `--waypoints` explicitly selects the historical staged
route; `--synchronous` and `--resume-lift3` select historical waypoint modes.
None performs final compensation trims. Continuous mode refuses combinations
with these options. Return remains disabled after the reported contact.

Updated FK predicts the saved HOVER midpoint at **(7.528, 1.960, 2.540) inches**.
That is **(+0.528, -0.040) inches** from pickup (7, 2) in XY. Joint arrival is
therefore not confirmation of block alignment. Preview reports target FK and
completion reports FK from an identified actual-angle snapshot. The existing
observed hover target is retained for this forward experiment.

The operator clarified that the block is **one inch tall** and the desired
approach should end at grasp height, followed by closing the jaws. The pickup command
uses the updated FK to solve a target at **(7, 2, 0.625)
inches**, assuming a side grasp with the pad midpoint 5/8 inch above the mat:

```sh
python3 -m bucke_arm.pickup
```

Candidate joints are approximately
`[37.2823, -20.7535, -117.1150, 45.6885, 0.08, 161.63]` degrees. The solver
holds J5/J6 and J2+J3+J4 fixed relative to HOVER to retain its nearly downward
flange tilt. J1 changes the heading slightly. `--grasp-height` changes the
candidate pad-midpoint height, above 1/2 inch and below the block's one-inch
height. The operator measured the fingers extending 1/2 inch below the pad
midpoint when pointing down, and described block width as slightly over one
inch. The former 1/2-inch grasp height gives zero nominal table clearance and
is now rejected. The revised 5/8-inch midpoint gives 1/8 inch nominal tip
clearance. This allowance is a planning choice, not verified clearance: tilt,
open-finger geometry, FK error and joint tracking can reduce it.

The operator confirmed physical clearance on November 18. Pickup execution is
now available separately from the original HOVER experiment. FK still models
a closed-pad point rather than open-finger geometry; the clearance confirmation
is a physical observation, not a result of the FK calculation.

From saved ready, with jaws already open wider than the block, use:

```sh
python3 -m bucke_arm.pickup --execute
```

Close other serial clients and use the exclusive-port check below, substituting
this pickup command for the HOVER command. At the local prompt type `PICKUP`.
The program checks stable, stationary, fault-free ready telemetry and live joint
limits, sends one all-joint command at speed 5, and records the measured approach.
After three settled samples within 1.5 degrees per joint it rechecks status and
angles. Actual-angle FK must be within 1/8 inch of (7, 2) in XY, with midpoint
height strictly above 1/2 inch and below the 1-inch block top, before jaw closure
is commanded. A failed check stops the sequence without closing the jaws or
issuing corrections. Starting jaws must be opened by the operator before the
run; the program does not assume an opening command has finished.

The recording uses direction `ready_to_pickup`, so the hover reverse-preview
tool refuses it. A complete trace means the approach was confirmed, not that a
grasp was sensed. The completion message says closure was commanded; check the
physical grasp. The arm remains at grasp height. Lifting, return, sweep and drop
are deferred. The assistant has not run hardware motion.

The historical screens below predate the updated mat calibration and do not
verify the new pickup candidate or current physical sweep.


## November 5 self-contact: return disabled

The user reported that direct ready-to-hover worked, but the gripper hit the
arm near the end of direct hover-to-ready. All public hover-return commands
are now disabled before any connection or movement. Moving back to the starting
angles does not guarantee reversal of the forward trajectory. The successful
forward run predates recording; its actual path cannot be reconstructed from
its endpoints. The former waypoint return is described below for the test record.

Default continuous forward sends one all-joint command, but now reads actual
joint angles and controller status throughout motion at a nominal 0.1-second
interval plus serial latency. This changes the Python wait mechanism from a
blocking SDK wait to monitored polling; it does not change the target or speed.
Every run saves a distinct JSON under `hover_trajectories/`, including incomplete
captures with a failure reason. Only confirmed final arrival marks a recording
complete. Capturing from ready is required. No recording was made of the old
successful run, and no new hardware motion was run by the assistant.

An offline reverse preview validates a complete recording and produces its
measured samples and time intervals in exactly reversed order, without angle
wrapping or interpolation:

```sh
python3 -m bucke_arm.resume_hover --preview-reverse hover_trajectories/FILE.json
```

The preview refuses invalid telemetry, wrong endpoints, incomplete recordings,
nonmonotonic time, and gaps over 0.5 seconds. It does not execute motion.
Finite sampled telemetry is not an exact continuous trajectory and does not
establish a safe swept volume between samples. Recorded return execution
remains unavailable until a successful measured forward trace and the contact
configuration can be evaluated. Leave the arm stopped following contact;
read-only `python3 pi_status.py` establishes the current controller state.

## Synchronous waypoints

The forward command accepts `--synchronous`. It calls `sync_send_angles` at
speed 5 for each waypoint, then checks arrival with three stationary, fault-free
angle samples and a two-second dwell. The blocking SDK wait pauses application
fault polling. The fixed Lift 3 forward resume also accepts `--synchronous`.

### Retired waypoint return

The offline return planner matches a stopped pose to padded boxes between
adjacent waypoints and lists the ready-side endpoint and the completed prefix
backwards. It uses signed angles, including J6. When segment boxes overlap, it
chooses the closest projected segment and breaks ties in favor of the earlier
segment. Poses outside every box are refused.

The direct ready-to-corrected-hover move was screened offline. Nominal equal
joint progress gave 45.4 mm minimum provisional gripper clearance, but independent
progress sampled at 0, 0.25, 0.5, 0.75 and 1 with 3-degree padding reached
5.8 mm below the empty mat. The waypoint options preserve the routed approach;
the direct `--continuous` option was subsequently enabled at the user's explicit
request, as documented above.
Synchronous SDK calls do not establish equal joint progress or collision-free
paths. Reverse waypoint segments cover the same joint-space boxes as forward
movement, retaining the provisional clearance limitations below. Reverse
hardware traversal and synchronous hover traversal remain unverified.

## November 5 stopped Lift 3 checkpoint

The user supplied three identical stationary, error-free samples at
`[33.75, -4.48, -98.78, 9.58, 0.43, 161.71]` after an invalid angle reading
during Lift 3. With that pose unchanged, use the same exclusive-port check
below, replacing the execution command with:

```sh
python3 -m bucke_arm.resume_hover --execute --resume-lift3
```

This checks all six joints within 0.5 degrees of that fixed readback and retries
Lift 3 before continuing the remaining 15 waypoints. It does not repeat the
ready approach or skip the partially completed waypoint. Fault, invalid-read,
completion and stop checks remain active. The additional
entry segment was screened with the provisional envelope described below,
sampling independent joint progress at 0, 0.5 and 1 with 2-degree padding:
minimum sampled empty-mat clearance 27.4 mm. Hardware execution is pending.

## Full route from ready

From the Pi's `~/buck-e-capstone/arm` folder, after copying the updated arm
folder from the primary Windows checkout:

```sh
python3 -m bucke_arm.resume_hover
```

This previews the default continuous move without opening a connection. The
staged route below requires explicit `--waypoints`. For a supervised run,
close the manual controller and other serial clients, then use:

```sh
(
    sudo -v || exit 1
    command -v fuser >/dev/null || exit 1
    sudo fuser -v /dev/ttyAMA0
    port_status=$?
    if [ "$port_status" -ne 1 ]; then
        echo "Port occupied or check failed; no motion started."
        exit 1
    fi
    python3 -m bucke_arm.resume_hover --execute
)
```

Type `HOVER` once after confirming the cube is out, jaws are open, the sweep is
clear, and an independent stop is within reach. Keep the original base, mat,
mounting, and gripper arrangement. The program does not stop services, power on,
release servos, or change reference settings. A Bluetooth bridge previously
competed for the port; the port must remain exclusive throughout the run.

Starting telemetry must be stationary, fault-free, stable within 0.5 degrees
across three samples, and within 3 degrees per joint of saved ready
`[-90, 0, 0, 0, 90, -135]`. The check repeats after confirmation and route-limit
queries. Unknown poses are refused rather than joined to the route.

With `--waypoints`, the 30-step approach reuses the prior initial-hover target, starting from saved
ready instead of the previous measured ready. Six steps each lift, shift and
lower to the prior corrected target:
`[34.75, -12.77, -93.56, 14.15, 0.08, 161.63]`. Speed is 5, waypoint timeout
8 seconds, with three stationary samples and a two-second dwell. Approach
tolerance is 2.5 degrees, subsequent stages 2 degrees. No final trims are issued; final error must be at most 1.5 degrees. Any telemetry failure,
fault, timeout, or interruption during execution requests stop. There is no
automatic retry, return, or descent beyond hover.

An offline screen sampled independent joint progress at 0, 0.5 and 1 across
each segment, using the prior provisional gripper envelope (axial -10 to
+137 mm, radial 75 mm, mat Z -14.2875 mm), padded by waypoint tolerances
(3 degrees at entry, 2.5 across the lift). Minimum sampled clearances above the
empty mat were 23.2 mm approach, 19.8 mm lift, 43.4 mm shift and 19.2 mm lower.
This is a sample of a provisional model, not continuous collision verification
or a bound on overshoot. Prior screens covered final trims with 2.5-degree
padding. Hardware replay has not been performed by the assistant.

The previous observed endpoint was `[33.66, -13.35, -94.13, 13, 0, 161.63]`.
The user saw the jaw midpoint over mat (7, 2) inches, while the flange axis
pointed toward about (6.5, 2). Replaying the target does not guarantee the same
physical endpoint. After completion, verify alignment; the next task is a side
view along mat Y to resolve tilt before planning descent. Keep the cube out.


## Raised J2-only test after the low pickup

The operator requested raising the former 0.625-inch target by one inch.
`python3 -m bucke_arm.raised_approach` previews a separate approach-only test
at nominal pad height 1.625 inches. It uses the latest stopped diagnostic
pose [36.65, -19.16, -117.59, 44.47, 0.08, 160.75] and changes only J2 to
approximately -11.9794 degrees. Other joint commands, including J6, are not
sent. Nominal XYZ becomes approximately (7.462, 2.223, 1.625) inches; this
single-joint lift follows an arc and deliberately does not recenter XY.

The initially considered all-joint raised endpoint was rejected: independent
joint progress could let J4 finish first and model a pad dip to 0.175 inches.
Raising an endpoint does not by itself eliminate an intermediate low sweep.
The J2-only test avoids this specific sequencing issue. An offline 101-point
screen predicts monotonic increasing midpoint height for this J2 interval;
that is a nominal model screen, not verified physical clearance or tracking.
J2 holding/response error and physical/model mismatch remain unresolved.

After transferring the updated package, `python3 -m bucke_arm.raised_approach
--execute` sends one J2 command at speed 5 only from that recorded pose. Three
stationary, fault-free start readings must match within 0.5 degrees and remain
stable within 0.25. Local/controller limits and the start are rechecked before
transmission. Actual motion is captured with direction
`stopped_to_raised_approach`. Three stationary samples within 1.5 degrees of
the target are required within 20 seconds; failures/interruption request stop.
The old grasp and return remain disabled. No closure, correction, parameter
change or automatic return is performed. This target is above the one-inch
block and cannot serve as its grasp pose. Measure the actual tip clearance
and mat XY after the raised test before selecting another move.


## Local high-hover orientation check

The operator confirmed the pad midpoint is already vertically over (7,2),
with height slightly above 2.25 inches, at measured joints
[33.92,-13.35,-93.95,12.83,0.52,160.48]. The gripper aims toward (6.5,2);
that is its tilted aiming point, not the midpoint's vertical projection.
Do not shift XY to compensate for that aiming coordinate.

`python3 -m bucke_arm.hover_alignment` previews a local candidate that preserves
the nominal midpoint at this observed pose and makes the nominal flange axis
vertical. `--execute` sends one command from this recorded pose only. J6 stays
at 160.48, no jaw command or automatic return occurs, and original low pickup
remains disabled. Remove the block: intermediate height can dip despite the
unchanged endpoint height. Check actual pad center, height and orientation
afterward. This is a local model-assisted alignment test, not a calibrated
pickup solution; the physical 1.625-inch approach and grasp are still pending.


The initial local alignment was refused because J5 had moved 0.87 degrees
between the saved hover reading and the test. The operator supplied three
new identical stationary/error-free samples. The current alignment checkpoint
is [34.27,-13.35,-93.95,13.0,-0.35,160.66], superseding the earlier checkpoint
above. The current calculated target is
[33.9587,-8.2415,-103.8620,22.1035,0,160.66]. The existing 0.5-degree start
tolerance remains unchanged; only `hover_alignment.py` needs transferring for
this checkpoint refresh. No alignment motion occurred before the refusal.


## Measured-reference raised approach trial

The operator measured pad midpoint (6.625,1.75,2.0) inches at actual joints
[34.18,-9.66,-104.23,20.47,-0.08,160.66]. Their initial 3-inch height was
explicitly corrected to 2 inches. This is the current physical reference.
The raised target (7,2,1.625) requires relative displacement
(+0.375,+0.25,-0.375) inches. `local_approach` uses that relative displacement
from the nominal current point rather than assuming absolute nominal XYZ is
physically accurate. It also targets nominal flange +Z downward and keeps J6
at 160.66. It does not establish accuracy of the local model between poses.

Desired reported joints: approximately
[34.2456,-13.8997,-108.4323,32.3320,0,160.66]. The one-command trial adds
J2 +0.590, J3 +0.368 and J4 +1.634 degrees based on observed nearby command
errors for those movement directions. This is bounded feedforward for this
trial only, not verified universal compensation. No firmware registers or
FK constants are changed. Commanded J4 and J2 are also checked against live
limits. The command is approximately
[34.2456,-13.3097,-108.0643,33.9660,0,160.66].

`python3 -m bucke_arm.local_approach` previews. `--execute` sends one all-joint
command at speed 5 only from that recorded stable start, with the existing
0.5-degree entry check and repeated live-limit/start checks. Completion checks
use the DESIRED reported angles, not the precompensated command. Captures keep
both arrays and direction `measured_hover_to_raised_approach`. Timeout prints
last recorded angles and signed target errors. No automatic retries, jaw
closure, return or parameter changes occur. The block stays out because
intermediate height can dip. The target is a raised approach, above the block,
not a grasp. Physical position and tilt still need checking after the trial.


### Local approach live-start update

The first local approach attempt was refused before movement at
[34.27,-9.66,-104.23,21.0,-0.52,159.96]. Fixed 0.5-degree matching was too
narrow for the observed between-snapshot settling. The command now samples
three stationary, fault-free readings inside a per-joint box spanning the two
operator-provided post-alignment observations, padded by 0.25 degrees.
It requires 0.25-degree stability across readings, repeats sampling after
limit queries, and checks the freshly captured pose immediately before sending.
Ready, the original hover, unrelated poses, faults and unstable samples refuse.

The physical anchor remains paired with the ORIGINAL measured joint snapshot;
it is not reassigned to live telemetry. The midpoint target stays fixed relative
to that measured anchor, while IK uses the live start's signed J6. This avoids
commanding J6 back to a stale value. Console and returned result include the
captured start, exact desired and precompensated command, and actual end angles.
Both command and desired targets are still checked against live joint limits.
This changes entry handling, not the physical goal, trial offsets, one-command
policy, completion tolerance, or disabled jaw/return behavior.


## Grasp from the physically verified raised approach

The operator has confirmed centered, almost vertical jaws at actual angles
[34.27,-14.06,-108.36,32.34,-0.52,159.96], with slight fingertip overlap on
the one-inch block. `python3 -m bucke_arm.grasp` previews a further relative
quarter-inch descent; `--execute` performs that one descent and commands jaw
closure only after joint/status and relative FK endpoint checks. Start with
open jaws and the block at (7,2). It samples a stable live start only near the
verified pose, rechecks limits and stability, retains wrist settings, and uses
the same bounded local directional feedforward trial offsets.

The local FK endpoint must show 1/8-3/8 inch of lowering, at most 1/8 inch of
XY drift per axis, and nominal flange tilt <=2 degrees. Failures stop without
closing or retrying. Actual physical placement is still the basis for the
start; the numerical checks do not supply collision sensing. Closure is a
command, not confirmation that a block is securely held. No automatic lift,
return or sweep follows. Original ready-to-low-pickup execution stays locked.


## Observed successful pickup and lift

The operator confirmed the local quarter-inch descent/closure gripped the
block. Two manual J2-only lifts at speed 5 followed: -12.00 reached -13.62
with uncertain corner clearance; -10.00 then reached -11.60 with the block
physically confirmed fully clear. Final actual held pose:
[34.18,-11.60,-110.39,37.17,-0.61,160.04].

`config/pickup_sequence.observed.json` preserves the successful phase commands,
reported poses and physical confirmations. This is a local pickup success,
not verification of a direct ready-to-grasp path, combined lift, sweep or drop.
Leave jaws closed at the held pose while choosing the drop destination.


## Combined local pick-and-place routine

The operator physically demonstrated pickup, lift, J1 sweep and landing near
(2.5,7). Final sweep command was J1 91 at speed 5; actual J1 was 90.08. This
is an approximate demonstrated drop point, not a calibrated exact (2,7).

`python3 -m bucke_arm.pick_and_place` previews. `--fake` simulates phases with
instant nominal state changes. `--execute` runs once, only from the physically
verified raised pickup pose centered over (7,2). It does not start from ready
or from the current drop pose. The block and fixed setup must match the
observations. It opens jaws, performs the guarded quarter-inch descent and
closure, waits for gripper stopping, lifts J2 with command -10, sweeps J1 with
command 91, then opens and waits. Speeds are 5 for arm motion and 20 for jaws.
No corrective retries, intermediate lift/sweep target commands, automatic
repeat, ready transition or return are included.

The routine consolidates manually demonstrated joint intervals; the combined
hardware run and gripper polling remain unverified on the unit. Gripper status
must be valid and stopped across three samples with at least two seconds since
waiting began; timeout/invalid status aborts. Gripper stopping is not force or
object sensing. Lift and sweep endpoints are checked before the next phase,
and every motion target is preflighted before opening jaws. A failed lift does
not sweep/release. A failed drop endpoint does not open jaws. The successful
manual observations remain distinct from the software-tested combined routine.


## New supervised drop-to-READY return

The operator requested a return to saved READY and proposed J6 -135 before
folding the arm. The observed drop checkpoint is
[90.08,-11.60,-110.39,37.17,-1.14,160.04]. J6 to -135 is -295.04 signed
degrees; it must not be wrapped into a 64.96-degree route through the limits.
A new route raises the empty tool first, then changes J6 before resetting
J4/J5. Single-joint stages at speed 5 are:

1. J2 to 0 (raise clear of the dropped block).
2. J6 to -135 (inspect the entire wrist/camera/gripper sweep).
3. J3 to 0 (raise elbow before reducing J4).
4. J4 to 0.
5. J5 to 90.
6. J1 to -90 (base turn last, with the tool raised).

`python3 -m bucke_arm.return_ready` previews without connecting. `--fake`
simulates states. `--execute` starts only near the observed drop checkpoint,
preflights every stage's live joint limits and requires three stable,
stationary, fault-free readings. Before each stage it pauses for the operator
to inspect clearance and press Enter. Q or any nonempty response cancels.
Only one joint is commanded per stage; each stage records its actual path,
checks three stationary samples within 2 degrees, and rechecks the completed
checkpoint before the next stage. Failures/interruption request stop, with no
automatic retry or arbitrary checkpoint join. Final READY uses its existing
3-degree saved-pose check. The jaws must already be empty/open; this return
does not issue jaw-position, power, calibration or reference commands.

`pick_and_place --execute --return-ready` offers the same supervised return
after release. Default pick-and-place still stops at drop. The old direct
hover-to-ready public method remains disabled.

The nominal tool/table screen used the historical provisional cylinder
(axial -10 to +137 mm, radius 75 mm) and updated level mat transform. Sampling
601 equal-progress points per single-joint stage gave minimum heights above
the mat about 27.8,54.3,54.3,357.2,357.2,357.2 mm respectively. These are
samples, not actual meshes or continuous clearance bounds. A counterexample
to J6-first then one all-joint reset is J4 reaching 0 while J3 remains -110.39:
the model tool envelope extends about 23.4 mm below the mat. Thus J6-first
alone does not remove the floor risk of asynchronous folding.

No full robot self-collision model or physical return traversal has been
verified. Local stage confirmations are retained for the first new traversal,
particularly because direct hover return previously caused self-contact.
Do not describe this staged return as physically proven until it has been
observed. Standalone return is appropriate at the current drop pose; the
combined pick-and-place command still starts only at the raised pickup pose.


The operator subsequently confirmed the staged return worked perfectly at
speed 5 throughout. At their explicit request, only the J6-to--135 stage now
runs at speed 10; all other stages remain speed 5. The faster J6 stage has not
yet been observed. Preview, local stage prompts and trajectory speed metadata
reflect the new profile. The route order and all confirmation/checks remain.


## Full supervised cycle from saved READY

`python3 -m bucke_arm.full_cycle` previews; `--fake` replays observed checkpoints
without hardware or motion physics; `--execute` runs once from saved READY.
The first combined hardware cycle has not yet been observed. Keep the block
out during the initial sweep, with empty jaws, until the local placement prompt.
The run opens and waits, executes the original single-command high HOVER,
replays the taught high-hover transfer, then performs the measured-reference
raised approach. It pauses there to place the one-inch block at (7,2) and
verify physical centering/alignment before descending and closing.

The taught transfer reuses the command that produced the observed intermediate
pose. It checks against the observed settled joint goal, not an ideal vertical
IK goal the prior trial did not actually reach. The local raised approach then
uses the same measured physical anchor and tested directional offsets, holding
live J6. There are no automatic compensation retries or a new direct low
READY-to-grasp shortcut. Motion is one command per demonstrated stage; the
first combined cycle retains the separate demonstrated approach stages.

After block placement, the guarded quarter-inch descent, closure wait, single
J2 lift, single J1 sweep and release run. The previously confirmed staged
return follows with its local clearance prompts. Return J6 uses 10; other arm
commands use 5. The full route reference targets are preflighted before opening
jaws; dynamic plans and start checkpoints are also checked at each phase.
Cancellation/failure stops without automatic recovery, release, restart or
another cycle. Initial pickup/deposit positions and the fixed setup must match
the demonstrated run. Physical placement confirmation is retained because the
absolute FK is provisional and the initial approach with a block present has
not been demonstrated.


The first full run reached the raised approach but stopped because the old
J5 signed-reference check rejected +0.70 versus the historical -0.52. The new
pose passes the existing modeled tilt and midpoint gates. Grasp now uses those
geometric gates for J5 while retaining the other joint bounds; drop return
accepts bounded J5 on either side of zero. No movement tolerance was increased.
After updating grasp.py and return_ready.py, the stopped raised approach can
resume via `python3 -m bucke_arm.pick_and_place --execute --return-ready` with
the block positioned and jaws checked centered/open. This skips the already
completed READY approach. Do not rerun full_cycle from the raised pose.


### Loaded gripper wait and post-grasp resume

A full run closed the jaws, then timed out waiting for the gripper before
lifting. A read-only probe afterward showed five stopped readings at partial
value 54; the status during the failed wait was not recorded. Loaded closure
now accepts valid stable partial-position telemetry after the minimum dwell,
without treating object contact as a demand to reach fully closed. It still
rejects invalid/changing/open readings and faults. Opening still requires the
stopped flag. These checks do not sense object identity or holding force.

If a block is already held after a stop, use
`python3 -m bucke_arm.pick_and_place --execute --resume-grasp --return-ready`
only with the known post-grasp pose and after confirming the block is between
the jaws at the local prompt. This starts with lift, not opening/descent.
Do not rerun full_cycle with a held block: a full cycle begins by opening jaws.
Update arm.py, grasp.py, return_ready.py and pick_and_place.py together for the
shared guard contract, loaded-wait logic and explicit resume option.


## Automatic return progression and paired J5/J1 finish

The operator reported the resumed sequence looked good, then explicitly asked
for no return confirmations and for J5/J1 to move synchronously. Default return
now advances automatically through checked J2, J6, J3, J4 stages and one final
all-joint READY command setting J5=90 and J1=-90 together. The other four goals
are the existing saved READY goals, not fresh measured following-error angles.
This avoids silently replacing prior holding goals with their tracking errors.
The SDK receives both moving axes in one command; identical physical arrival
times are not established. The final READY checks remain mandatory.

All route/limit, stable-start/checkpoint, fault, trajectory and completion checks
are retained. Q/Enter return prompts are absent by default in return_ready,
pick_and_place --return-ready and full_cycle. Optional --step on the standalone
return remains for diagnostics. Pickup block-placement confirmation is unchanged.
J6 remains speed 10 and the paired finish speed 5. The old direct hover return
remains disabled. No automatic repeat is added.

The paired final stage and default prompt removal are software-tested changes,
not a new observed hardware traversal. The earlier separate-stage return was
physically confirmed. Nominal table height of the paired J1/J5 finish is high;
that model is not proof of external-obstacle or robot self-clearance.


## Block pre-positioned before the cycle

The operator reported the updated cycle worked and asked to remove the need
to place the block mid-run. `full_cycle --execute` now assumes the one-inch
block is already at (7,2), proceeds through the existing checked raised pose,
and picks it up without operator input. Only the placement callback selection
changed: the arm trajectories, targets, speed profile, geometry/stability/fault
checks, contact-aware closure, lift, release and automatic return are unchanged.

This does not add block detection, feeding, physical repositioning between
cycles or automatic repeats. After a drop near (2.5,7), another run requires a
block at the pickup location again. `--place-at-pickup` keeps the old block-out
approach and placement pause as an explicit manual diagnostic option.

The approach has previously been physically run with the block absent. The
first pre-positioned-block run must be watched for unintended contact before
the grasp; software tests do not verify clearance past the newly present block.
No new claim of swept collision clearance is made by removing the pause.
