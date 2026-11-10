# Resume the November 3 hover

## November 5 self-contact: return disabled

The user reported that direct ready-to-hover worked, but the gripper hit the
arm near the end of direct hover-to-ready. All public hover-return commands
are now disabled before any connection or movement. Moving back to the starting
angles does not guarantee reversal of the forward trajectory. The successful
forward run predates recording; its actual path cannot be reconstructed from
its endpoints. The former waypoint return is described below for the test record.

`--continuous` forward still sends one all-joint command, but now reads actual
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
completion, final correction, and stop checks remain active. The additional
entry segment was screened with the provisional envelope described below,
sampling independent joint progress at 0, 0.5 and 1 with 2-degree padding:
minimum sampled empty-mat clearance 27.4 mm. Hardware execution is pending.

## Full route from ready

From the Pi's `~/buck-e-capstone/arm` folder, after copying the updated arm
folder from the primary Windows checkout:

```sh
python3 -m bucke_arm.resume_hover
```

This previews the route without opening a connection. For a supervised run,
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

The 30-step approach reuses the prior initial-hover target, starting from saved
ready instead of the previous measured ready. Six steps each lift, shift and
lower to the prior corrected target:
`[34.75, -12.77, -93.56, 14.15, 0.08, 161.63]`. Speed is 5, waypoint timeout
8 seconds, with three stationary samples and a two-second dwell. Approach
tolerance is 2.5 degrees, subsequent stages 2 degrees. At most two final trims
offset a nominal target by up to 0.5 degrees; final error must be at most
1.5 degrees and each trim must improve maximum error. Any telemetry failure,
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
