# Pi configuration and read-only verification

## Joint limit readback, November 9, 2026

The operator's Pi terminal capture showed these values from
`get_joint_min_angle()` and `get_joint_max_angle()`. They matched the
`MyCobot280` limit table in pymycobot 4.0.6, and SDK argument validation
accepted both endpoints for all six joints.

| Joint | Controller and SDK bounds (degrees) |
| --- | --- |
| J1 | −168 to +168 |
| J2 | −140 to +140 |
| J3 | −150 to +150 |
| J4 | −150 to +150 |
| J5 | −155 to +160 |
| J6 | −180 to +180 |

Buck-E's prior command table used ±135° for J2, ±145° for J4 and ±165° for
J5. Buck-E uses tighter operator-specified caps for J2 (±135°) and J4 (±145°)
in the command table and offline preview. The other joints match the readback.
This diagnostic queried limits and ran local validation only. Physical travel
and clearance at the endpoints remain unverified.

## Direct hover-return self-contact reported, November 5, 2026

The user reported that one-command ready-to-hover worked well, but direct
hover-to-ready caused the gripper to hit the arm near the end, when the gripper
changed orientation. This is operator-reported contact; its joint angles,
precise contact surfaces, stopping state and damage status were not supplied.
An earlier return screenshot stopped on `error=-1`, an invalid query response;
that screenshot does not identify the later contact pose or prove a controller
collision fault. Do not treat saved endpoint angles as a reversible trajectory.

Public hover-return execution is disabled in the local code. Forward continuous
motion now records measured joint samples and status; reverse preview validates
and reverses those samples offline only. The successful earlier forward run
was not recorded, so its actual path is unavailable. No robot movement was
performed by the assistant. Current post-contact telemetry and a successful
measured forward trace are required before planning a corresponding return.
See [hover trajectory status](resume_hover.md). The saved standby/ready paths
documented separately below are not hover recovery routes.

## Arm class consolidation, October 28, 2026

At the user's request, reusable hardware behavior now lives in `RealArm` and
simulation in `FakeArm`, both in `bucke_arm/arm.py`. The former read-only
interface, HardwareArm adapter and factory were removed. `joint_control.py`
is now a small manual testing menu that calls the classes. Saved-pose starting
state, limits, synchronous command and final telemetry checks live in RealArm.
FakeArm tracks commanded joint/gripper state without modeling physical motion.
The offline preview now shares the active ready target `[90, 0, 0, 0, 90, -135]`.
The prior negative-J1 target and old architecture described below are historical.

## Synchronous transitions confirmed and experiments retired, October 28, 2026

After running the synchronous controller, the user reported "Great both work
safely!" for standby-to-ready and ready-to-standby. This records success as
reported by the operator; no new endpoint readings or trajectory measurements
were provided. The current saved poses and speed setting 5 remain unchanged.

At the user's request, the sequential test mode, resting CSV logger, J5 stepping
helper, J4 recovery helper, and their dedicated tests were removed. Synchronous
transition regression tests, read-only diagnostics and offline ready-planning
tests remain. The sections below preserve historical evidence and may refer to
removed scripts or superseded flags; use the arm README for current commands.

## Synchronous saved-pose testing enabled, October 28, 2026

The user subsequently stated "I verified both paths" and clarified that the
next requested test is synchronous movement using `sync_send_angles()`.
The controller now records both path-verification flags as true based on that
user report. Normal R/S commands use the saved poses below at SDK speed setting
5, timeout 120 seconds, followed by an independent final telemetry check.
No synchronous hardware result, trajectory trace, or endpoint measurements have
yet been supplied; user-reported path verification is not an independently
measured all-joint trajectory. The earlier sequential-only preparation below
is historical; `--sequential-test` remains available as an option.

The SDK blocks application fault polling while waiting and returns 1 even on
timeout. Final stationary/error-free telemetry within the existing 3-degree
pose tolerance is required before the application reports success. See the
README for the current invocation and implementation limits.

## Sequential saved-pose test prepared, October 28, 2026

The user confirmed use of the poses currently in `joint_control.py`: standby
`[0, 0, 90, 90, 90, -135]` and ready `[90, 0, 0, 0, 90, -135]`.
They stated the physical route should be fine and requested slow sequential
movement first, with all-joint movement considered afterward. This is user
assessment and test intent, not completed physical path validation.

`python3 -m bucke_arm.joint_control --sequential-test` offers R/S tests at SDK
speed setting 5. The candidate ready sequence is J1, J3, J4, with the return
using J4, J3, J1. Every joint requires fresh local confirmation and completion
monitoring before another step is offered. No hardware movement was performed
by the assistant. Both all-joint verification flags remain false. See the arm
README for the invocation, monitoring criteria, and cancellation behavior.

> Cleanup, October 21, 2026: the old J1/J5 test helpers, J1 positioning
> helper, and fake-serial packet-check script have been removed. References
> to those scripts below are historical; use the current arm README for retained tools.

Recorded from the user's Pi command output and screenshots shared on October 13, 2026. These observations establish the environment and basic communication. They do not establish calibration or safe movement.

## User-reported J6 endpoint traversal, October 21, 2026

The user tested J6 by positioning it at -180 degrees, then commanding +180
degrees, and observed movement through zero. This is user-reported physical
path evidence for that tested transition, rather than a short wrap across the
endpoint boundary. No sampled trajectory, endpoint accuracy, speed, or other
joint behavior was supplied. It does not establish the rest-to-ready route.

The corrected ready target is `[-90, 0, 0, 0, 90, -135]` degrees in J1-J6 order.
A rest-to-ready function still needs a specified rest pose and reviewed movement
sequence. Use ordinary signed joint differences when planning this observed J6
behavior; do not substitute a shortest-angle modulo-360 calculation.

## Latest starting-pose evidence and deferred diagnosis

The user's latest pi_status.py screenshot showed three identical samples:
`[166.72, 0.70, -0.52, -0.79, 89.12, -134.12]` degrees, moving 0, error 5.
Servo status was `[0, 0, 0, 0, 0, 0]`, voltages
`[12.1, 12.1, 12.1, 7.9, 7.6, 7.5]`, and temperatures
`[39, 34, 30, 36, 46, 45]` C. The user subsequently reported setting J1 to
-90 degrees and corrected the ready target above. No fresh six-joint telemetry
or error-clear result was supplied after that adjustment. Error diagnosis is
deferred at the user's request while they are away from the arm. Offline
planning and simulation can proceed; hardware execution and route validation
remain pending. Do not treat the earlier screenshot as the current pose.

## Confirmed environment

| Item | Observed value |
| --- | --- |
| Arm model | Elephant Robotics myCobot 280 Pi, confirmed by the user |
| OS | Ubuntu 20.04.4 LTS, Focal Fossa |
| Architecture | aarch64 / ARM64 |
| Kernel | 5.4.0-1073-raspi |
| Python | 3.8.10, `/usr/bin/python3` |
| pymycobot | 4.0.6 |
| pyserial | 3.5 |
| Package directory | `/home/er/.local/lib/python3.8/site-packages` |
| Linux user | `er` |
| Serial device | `/dev/ttyAMA0` |
| Serial alias | `/dev/serial0` points to `ttyAMA0` |
| Device permissions | `crw-rw----`, owned by `root:dialout` |
| User access | `er` belongs to `dialout` |
| Tested baud rate | 1,000,000 |

No `/dev/ttyUSB*` or `/dev/ttyACM*` devices matched the inventory command. `/dev/serial1` points to `ttyS0`; this was not the tested arm connection.

## Installed API inspection

The installed `MyCobot280` constructor signature is:

```python
(port, baudrate='115200', timeout=0.1, debug=False, thread_lock=True)
```

The library exports `PI_PORT = '/dev/ttyAMA0'` and `PI_BAUD = 1000000`. Supply the tested baud explicitly because the constructor default differs.

The captured constructor opens a serial connection, sets RTS to false, and waits 1.5 seconds. It contains no explicit motor-power or movement call. A later screenshot showed that the parent `CommandGenerator` constructor initializes logging and configuration. The methods inspected in that screenshot use:

```python
get_angles(): self._mesg(ProtocolCode.GET_ANGLES, has_reply=True)
get_coords(): self._mesg(ProtocolCode.GET_COORDS, has_reply=True)
```

The coordinate method describes a base-referenced `[x, y, z, rx, ry, rz]` result. The orientation convention and configured tool offset have not been independently verified. This inspection covers the captured functions, not every SDK code path.

## Standalone telemetry test passed

The user ran a standalone Python script that opened the tested connection, requested angles and coordinates three times, checked for six finite numeric values in each list, and closed the serial port. All three samples were identical:

```text
Joint angles: [-2.63, 8.7, -26.36, 130.34, 99.14, 9.31]
Tool coordinates: [-54.8, -15.8, 304.4, 122.17, -74.02, 142.64]
PASS: three valid telemetry samples received.
```

These are historical readings transcribed from the user's screenshot. They are not home poses, calibration values, or approved motion targets. Identical samples demonstrate repeatable reported values over this short test; measurement accuracy is not established.

## Repository deployment and packaged adapter verified October 21, 2026

- Running `python3 -m bucke_arm status --backend hardware` from the Pi's home directory returned `No module named bucke_arm`. The repository had not yet been cloned onto the Pi.
- HTTPS cloning encountered authentication failure and later a 403 response containing `Write access to repository not granted`. Successful GitHub authentication on the Pi has not been confirmed. The exact credential or permission cause is unresolved.
- The user subsequently transferred the repository over SSH, reported all Pi unit tests passing, and provided screenshots of successful simulated and packaged hardware status.
- The latest hardware screenshot reports joint angles `[43.06, -14.58, 11.07, 153.8, 79.8, -44.91]` and tool values `[4.9, -20.7, 280.9, 84.77, 15.53, -129.45]`. These are historical telemetry, not approved motion targets.
- A follow-up screenshot reports firmware 7.3, error information 4, and angles `[43.06, -14.58, 11.07, 153.8, 79.8, -46.0]`. Manufacturer documentation identifies error 4 as a joint-4 limit error and lists J4 limits of -150 to +150 degrees.
- The user reports manual positioning, no dedicated release button, and a base power switch. The user confirmed secure suction mounting, removal of the nearby bottle, cable slack, and access to that switch. These are user confirmations, not independent inspections or emergency-stop certification.
- The user explicitly authorized releasing and re-engaging only J4 for manual recovery. `bucke_arm.recover_joint4` is a separate, interactive helper; it has not been run on hardware. The ordinary status adapter remains read-only. Recovery is pending; no calibration change or powered target-position movement is authorized by this recovery consent.
- Subsequent recovery screenshot: initial angles `[43.06, -14.58, 11.07, 153.36, 80.3, -45.7]`, error 4, release response 1. At `HOLD J4`, angles were `[42.97, -14.58, 11.07, 117.42, 81.21, -45.7]`. The original helper exited at its monitoring-window check before sending a focus command. Holding torque has not been confirmed restored. The user reports keeping the chosen physical pose stationary; physical angle and encoder calibration have not been independently verified.
- The updated helper includes a separately selected `--resume-hold` mode, which never releases a joint or commands a target position. It checks stationary, stable, in-limit telemetry and requires `HOLD J4` before requesting torque. Offline tests include the screenshot's exact pose. Hardware execution of the update is pending.
- The user subsequently confirmed J4 physically holds after resume-hold. A screenshot reports angles `[43.06, -14.5, 11.16, 119.53, 81.03, -45.7]` with error 4. After one explicit error-clear operation, another screenshot shows error 0 on three subsequent queries and identical before/after angles. This confirms the observed recovery and absence of a reported error during those checks; it does not verify calibration or collision clearance.
- The user authorized preparing one J1 movement from 43.06 to fixed target 44.06 degrees, speed setting 5, with no automatic return. `bucke_arm.test_joint1` implements that separate supervised test and passes offline tests; hardware execution is pending. No broader motion sequence is authorized.

The first J1 attempt failed: initial angles were `[43.06, -14.5, 11.16, 119.53, 81.12, -45.7]`;
the first printed post-command sample was `[43.06, -14.5, 11.16, 120.67, 82.88, -45.7]`.
The user reported apparent J5 movement. J1 had not changed in that sample, while
J4 and J5 changed by +1.14 and +1.76 degrees. The guard requested stop and received
response 1. This does not prove which physical servo received the command or
identify the cause.

The subsequent read-only diagnostic screenshot confirms SDK 4.0.6 at
`/home/er/.local/lib/python3.8/site-packages/pymycobot/mycobot280.py`, firmware 7.3,
and installed `send_angle` forwarding its supplied ID unchanged to SEND_ANGLE.
Three samples report J1 43.33, J2 -14.5, J3 11.16, J4 120.67, J6 -45.7; J5 is
82.8, 82.88, and 82.8 respectively. Each sample reports moving 0 and error 0.
J1 therefore differs by +0.27 degrees from its pre-test value; the first printed
sample was not the final pose. J4/J5 changes remain unexplained. No physical
joint remapping, command-delivery cause, or motor fault has been established.

Next: read-only diagnostics of stationary telemetry, error state, and the actual
installed SDK command implementation. The local J1 helper's hardware CLI is now
disabled before connecting; the Pi copy must be replaced to apply that block.
No retry or alternate joint-ID test is approved by the previous one-test consent.
GitHub authentication was still unverified at this stage. File transfer over
SSH allowed execution on the Pi.

Additional user evidence and diagnostic preparation:

- The simple `pi_status.py` ran on the Pi. Three identical angle samples were
  `[43.33, -14.5, 11.16, 120.67, 82.88, -45.7]`; each reported moving 0 and error 0.
  Servo status was `[0, 0, 0, 0, 0, 0]`, voltages `[12.0, 12.0, 12.0, 7.9, 7.7, 7.7]`,
  and temperatures `[38, 35, 30, 42, 61, 34]` Celsius. No validated internal-servo
  thermal threshold or normal per-joint voltage range has been established.
- A user photo shows the robot label `myCobot 280-PI`, `DC 12V 60W`. This is the
  robot's labeled input rating, not an independent measurement or adapter-label
  verification. Generic 8.4 V model documentation must not override this label.
- The user reports `fuser -v /dev/ttyAMA0` listed nothing while our scripts were
  finished. This only covers visible processes at that moment.
- The isolated installed-SDK encoder produced `fe fe 06 21 01 11 36 05 fa`, ID 1.
  The user locates the observed movement immediately behind the gripper joint.
  Exact physical ID mapping and cause remain unresolved.
- At the user's request, `pi_command_check.py` now exercises the actual
  `MyCobot280.send_angle` method with `serial.Serial` mocked before construction.
  A temporary local installation of pymycobot 4.0.6 captured exactly one expected
  packet without a real serial connection. This was not run on the user's Pi.
- `pi_rest_check.py` prepares read-only mode/enable queries and 30 resting samples
  with a timestamped CSV and peak-to-peak summary. It passes offline tests; its
  execution on the Pi remains pending. No further physical movement is enabled.

## Pi diagnostic results supplied October 21, 2026

The user reports `pi_command_check.py` printed PASS on the Pi. The full command
check output was not supplied. This checks the SDK path with a fake serial port;
it does not establish controller behavior or physical joint mapping.

Two screenshots show a completed `pi_rest_check.py` run:

- Firmware 7.3; free mode 0; enabled replies `[1, 1, 1, 1, 1, 1]`.
- All 30 angle samples: `[149.15, -14.58, 11.16, 122.78, 86.48, -45.7]`.
- Every sample reports moving 0 and error 0; all six peak-to-peak changes are 0.0.
- Initial and final servo status: `[0, 0, 0, 0, 0, 0]`.
- Initial voltages: `[12.1, 12.1, 12.0, 7.7, 7.2, 7.7]`.
- Initial and final temperatures: `[38, 35, 30, 43, 66, 34]` Celsius.
- Reported log: `/home/er/buck-e-capstone/arm/pi_rest_20261021_123858_651709.csv`.
  The CSV itself has not been supplied.

Reported readings were stable during this run. J1 is 105.82 degrees above the
earlier 43.33-degree reading; J4 and J5 also differ from the earlier resting
snapshot. The intervening physical positioning, commands, and power history
have not been established. These differences cannot be attributed to this
read-only test. J5 reports 66 C versus the earlier 61 C; no validated threshold
is available in the project. Neither diagnostic clears the unresolved movement
issue or establishes a safe startup pose. Further powered movement remains
disabled pending diagnosis and a reviewed, explicitly authorized plan.

## Newly authorized single J1 test, October 21, 2026

This section supersedes the earlier prohibition on retrying under the previous
one-test consent. It preserves the observed history and does not resolve the
cause of the earlier unexpected changes.

The user supplied a new `pi_status.py` screenshot. Firmware is 7.3; all three
angle samples are `[149.15, -14.58, 11.16, 122.78, 86.48, -45.7]`, each with
moving 0 and error 0. Servo status is all zero, voltages are
`[12.1, 12.1, 12.1, 7.8, 7.0, 7.7]`, and temperatures are
`[38, 35, 30, 43, 66, 34]` Celsius. No internal-servo temperature threshold for
this exact hardware revision has been verified or inferred from ambient limits.

After reviewing a plan for one J1 move from 149.15 to fixed target 148.15 degrees
at API speed setting 5, with no automatic return, the user reconfirmed secure
mounting, a clear movement area, and access to the power switch, and explicitly
authorized proceeding. The new `bucke_arm.test_joint1` helper implements only
that supervised plan, with a disconnected default, explicit hardware flag,
operator prompt, state checks before/after the prompt, and one asynchronous
target command. The asynchronous path in the manufacturer's SDK 4.0.6 source
writes once rather than using the synchronous reply/retry loop:
[MyCobot280 source](https://github.com/elephantrobotics/pymycobot/blob/v4.0.6/pymycobot/mycobot280.py),
[command generator](https://github.com/elephantrobotics/pymycobot/blob/v4.0.6/pymycobot/generate.py).

The helper requests stop on unexpected angle changes, bad telemetry, faults,
interruption after a movement attempt, or failure to confirm the target within
its monitoring window. Its numerical tolerances are experimental abort criteria,
not validated safety or calibration limits. SDK queries are synchronous, so
software cannot guarantee prompt stopping. Temperature readings are displayed
without claiming thermal clearance. A telemetry pass still requires the user to
confirm that only the physical base joint moved.

All 43 offline unit tests passed locally, including no-connection preview,
cancellation, preflight refusals, a state change while awaiting confirmation,
one-command success, other-joint changes, communication failure, write failure,
interrupt, and timeout. No physical test has been executed by this update.
SSH to `er@172.30.2.214` reached password authentication but noninteractive login
was denied; copying the updated helper and running it on the Pi remain pending.

## Result of the newly authorized J1 test, October 21, 2026

The user ran the helper and reported no visible movement. The supplied screenshot
shows repeated angles `[149.15, -14.58, 11.16, 122.87, 86.48, -45.7]`, moving 0,
error 0, zero servo status, and temperatures `[38, 35, 30, 44, 66, 34]` Celsius.
The beginning of the run and the operator prompt are outside the screenshot.
The displayed timeout message corresponds to the helper reaching its monitoring
loop after attempting its single asynchronous target command.

The helper timed out with `Target was not confirmed within the monitoring
window`, requested stop, and printed stop response 1. Its final readback has the
same angles, moving 0, and error 0. The target 148.15 degrees was not observed;
this is an unsuccessful movement test, not a telemetry pass or a movement too
small to see. The screenshot does not establish command reception or acceptance
by the controller. The asynchronous SDK call does not read a movement-command
acknowledgement. No cause has been established and no retry, speed increase, or
larger target has been authorized by this result.

Next: query the controller's power state, command refresh mode, and configured
J1 minimum/maximum angles using read-only SDK calls. These add controller-state
information without changing modes, torque, limits, or position. Do not infer a
need for `power_on`, a mode change, or a limit change from an unexpected reply.
The preceding preparation section's pending hardware-run statement is historical;
this screenshot confirms execution of the helper's monitoring/abort path.

## Controller-state query results, October 21, 2026

The next user screenshot shows the read-only query results:

- Power state: `1`.
- Command refresh mode: `1`.
- J1 configured minimum: `-168.0` degrees.
- J1 configured maximum: `168.0` degrees.

The planned 148.15-degree target is within those reported configured bounds.
Refresh mode 1 prioritizes the latest command; that result alone does not
establish that monitoring queries cancelled the movement. Power state 1 reports
controller power status, not independent verification of motor supply or torque.
These results do not establish why J1 did not move.

Before choosing another motion attempt, inspect the installed SDK serial-write
function without opening a connection. The upstream v4.0.6 `common.write` function
returns silently when `isOpen()` is false and catches/logs `SerialException`
without re-raising it. Its async movement path returns without reading the
movement-command acknowledgement. Consequently the helper reaching its monitoring
loop does not prove a successful serial write or controller acceptance. This is
a software observation, not evidence that either failure occurred on the Pi.
Next requested evidence is the installed `common.write` source and source path;
the inspection imports the library but never constructs a robot client.

## Installed write source and traced helper preparation, October 21, 2026

Two further user screenshots show the installed write implementation at
`/home/er/.local/lib/python3.8/site-packages/pymycobot/common.py`. Its serial branch
matches the upstream behavior: it returns if the port is closed, writes without
checking the byte count, and catches/logs `serial.SerialException` without
re-raising. This confirms the inspection result but not a write failure during
either physical attempt.

The local J1 helper now temporarily wraps only its movement write to surface
closed-port/serial failures, require a complete nine-byte host write, and log
the actual packet. It rejects any packet other than
`fe fe 06 21 01 39 df 05 fa` (J1, absolute 148.15 degrees, speed 5) and blocks a
second movement write. The wrapper is removed even on failure, before stop or
further queries. The installed SDK is not modified. After a complete write, a
read-only SDK reader call attempts to capture a raw reply for approximately
0.25 seconds plus serial-read timeout before another query clears the input.
Missing replies do not cause retransmission. Raw replies and byte counts are
not proof of achieved position or physical movement.

All 45 offline unit tests passed. A separate check using the actual pymycobot
4.0.6 package and a mocked serial constructor verified the exact packet,
one-write behavior, fake reply capture, and visible closed-port, short-write,
and serial-exception failures. That check opened no real serial connection.
The traced helper still requires the hardware flag and `MOVE J1` consent for
one further attempt at the same target and speed. This revision has not been
executed on the Pi; no new physical movement result is claimed.

## Traced speed-5 attempt result and proposed speed comparison, October 21, 2026

The user supplied two screenshots of the traced helper run:

- Pre-command J1 reading: 149.15 degrees.
- TX: `fe fe 06 21 01 39 df 05 fa`; host write accepted 9 of 9 bytes.
- RX: `fe fe 03 21 01 fa`, a reply with the movement opcode 0x21 and result 1.
- Final angles: `[149.06, -14.58, 11.16, 122.87, 86.48, -45.7]`.
- Displayed final status: moving 0, error 0, zero servo status, temperatures
  `[38, 35, 30, 45, 66, 34]` Celsius.
- Target-confirmation timeout, stop response 1, and unchanged final readback.

This establishes a complete host write and a matching command reply. Result 1
is consistent with the documented SDK response; it is not proof of achieved
position. J1's displayed change was -0.09 degrees, leaving it 0.91 degrees above
target. That small change alone does not distinguish physical motion from
encoder quantization or measurement variation. No other-joint changes appear
between the supplied pre-command/final readings, but the full run was not supplied.
The earlier swallowed-write-error possibility does not explain this complete
write/reply trace. The cause of the failure to reach target remains unknown.

A comparison is prepared: retain the fixed J1 target 148.15 degrees, starting
near the latest 149.06-degree reading (approximately a 0.91-degree decrease),
and change only the API speed from 5 to 10. Low-speed behavior is a hypothesis,
not a diagnosed fault. No calibration, torque, refresh-mode, or other joint
target changes are included. The helper accepts only speeds 5 and 10, defaults
to 5, displays the selected speed before its motion prompt, and checks the
expected outgoing packet with that speed byte. Speed 10 requires `--speed 10`
and a new operator `MOVE J1` confirmation for one attempt. It has not been run
on hardware and is not authorized merely by the latest result screenshots.

The helper also queries the documented read-only `read_next_error` joint
diagnostics before and after its operator prompt; invalid or nonzero replies
block movement. All 48 offline unit tests passed, including the speed-10 packet,
single-command behavior, rejection of other speeds, and new diagnostic faults.
A separate test with actual SDK 4.0.6 and a fake serial port verified exactly
one speed-10 packet `fe fe 06 21 01 39 df 0a fa` without a real connection.

## User-requested five-degree J1 test, October 21, 2026

The user explicitly asked for a five-degree rotation instead of the one-degree
test. The prepared plan is one decreasing J1 command from the latest reported
149.06 degrees to fixed absolute target 144.06 degrees at speed setting 10, with
no automatic return or retry. It supersedes the unexecuted one-degree speed-10
comparison; no result of that comparison has been supplied. Prior setup clearance
confirmations persist. This authorization does not cover other joints, calibration,
torque/mode changes, or a larger movement sequence.

The helper now uses the latest six-angle snapshot as its reviewed starting pose,
retains its existing start/stability checks and monitoring/abort thresholds, and
defaults to speed 10. A changed start is refused rather than silently adapting the
fixed target. The outgoing packet at speed 10 is
`fe fe 06 21 01 38 46 0a fa`. Hardware execution remains pending.

All 50 offline tests passed, including the new target packet, refusal to treat a
one-degree intermediate position as success, and refusal of the old proposed
target as a new starting pose. A separate actual-SDK-4.0.6 check with mocked serial
confirmed exactly one expected 144.06-degree, speed-10 packet. No real port was
opened. This verifies software behavior only; the cause of prior unsuccessful
physical attempts remains unresolved.

## Five-degree attempt blocked by diagnostic format, October 21, 2026

The user's screenshot shows the updated five-degree plan at speed 10 followed
immediately by `Invalid joint diagnostics: [0, 0, 0, 0, 0, 0, 0]`. The helper had
assumed six diagnostic values. This exit occurred during preflight, before the
operator prompt or a movement command; no five-degree motion was attempted by
this invocation.

The diagnostic validator now accepts six or seven integer entries and requires
every entry to be zero, both before and after the operator prompt. It prints the
raw reply and does not assign an unverified meaning to the seventh entry. Other
lengths, invalid types, or any nonzero entry still block motion; six-value joint
angle, servo-status, and temperature validation is unchanged. Target 144.06,
speed 10, tracing, and monitoring remain unchanged.

All 53 offline tests passed, including the exact seven-zero reply through the
single-command success path, faults in every slot of both accepted lengths,
malformed replies, and a seventh-slot fault appearing after confirmation. The
format fix has not yet been deployed or run on the Pi. The existing five-degree
authorization remains applicable because the blocked invocation sent no motion.

## Five-degree test produced observed movement, October 21, 2026

After transferring the diagnostic-format fix, the user reported "It moved" and
supplied the final helper output. The screenshot shows final angles
`[144.93, -14.58, 11.16, 122.87, 86.48, -45.7]`, moving 0, error 0, zero servo
status, and temperatures `[38, 35, 30, 45, 66, 34]` Celsius. The helper timed out
because target 144.06 was not confirmed, requested stop (response 1), and read
back the same angles with moving 0 and error 0.

Relative to the last reviewed J1 reading of 149.06 degrees, the final reported
change is -4.13 degrees. The endpoint is 0.87 degrees above target, outside the
existing 0.25-degree pass tolerance. The starting samples and TX/RX trace are
outside this screenshot, so -4.13 is a comparison with the prior supplied
snapshot rather than a measurement from a complete current-run log. J2-J6 match
that prior snapshot at the endpoint; the screenshot does not establish their
entire motion history. The user has not yet specified which physical joint
moved. This establishes observed movement and a changed J1 reading, but not a
target-position pass, calibrated accuracy, or a verified physical joint mapping.

Do not widen the pass tolerance to turn this timeout into success, or automatically
retry, return, or increase the movement. The next clarification is whether the
observed motion was only at the base joint. Further motion needs a specific plan;
the completed five-degree attempt does not authorize a larger sequence.

## Physical J1 isolation confirmed, October 21, 2026

The user subsequently confirmed "Just J1 moved" for the completed five-degree
test. This supplies the previously pending physical observation: only J1 moved
during that attempt. Together with the final reported J1 144.93 degrees and
unchanged endpoint values for J2-J6, this establishes a supervised isolated J1
movement milestone for that attempt. The 144.06-degree target was not reached
within the existing tolerance; the 0.87-degree endpoint shortfall and repeatability
remain unresolved. This confirmation does not explain earlier unexpected joint
changes or authorize another motion sequence. No motor command, pass-tolerance
change, or calibration change was made in response to this confirmation.

## Fresh baseline and opposite-direction check prepared, October 21, 2026

Following the proposed J1 direction/repeatability check, the user supplied a new
`pi_status.py` screenshot. All three samples report
`[144.93, -14.58, 11.16, 122.87, 86.48, -45.7]`, moving 0 and error 0. Firmware
is 7.3 and SDK 4.0.6. Servo status is zero, voltages are
`[12.1, 12.1, 12.0, 7.8, 7.3, 7.7]`, and temperatures are
`[38, 35, 30, 45, 66, 34]` Celsius. The readings match the prior endpoint.

The prepared next plan is one J1 increase from reviewed 144.93 to fixed target
149.93 degrees at speed setting 10, with no automatic return/retry. It is a
five-degree opposite-direction comparison, not an exact return to the earlier
149.06-degree start. The fresh starting-pose gate is updated, and the monitoring
interval now uses the lower/upper of baseline and target, retaining the same
0.25-degree margin and all other fault/other-joint abort checks. The operator
prompt explicitly displays the increase and fixed target. Execution requires
the hardware flag and `MOVE J1` consent for this displayed plan.

All 53 offline tests passed, including single-command target success, rejection
of opposite/wrong-direction travel and overshoot, seven-entry diagnostics, and
refusal to treat a one-degree intermediate position as success. An actual SDK
4.0.6 check with fake serial generated exactly one expected packet
`fe fe 06 21 01 3a 91 0a fa`. No real port was opened. Hardware execution and
confirmation of the physical joint/direction and endpoint remain pending. This
single comparison cannot establish broad repeatability, calibration, or a safe
automatic startup pose.

## Opposite-direction J1 movement observed, October 21, 2026

The user reported "It moved with no other joint movement" for the opposite-
direction test. The supplied final output shows angles
`[149.15, -14.58, 11.16, 122.87, 86.48, -45.7]`, moving 0, error 0, zero servo
status, and temperatures `[38, 35, 30, 45, 66, 34]` Celsius. The helper timed out
without confirming target 149.93, requested stop (response 1), and read the same
angles with moving 0 and error 0. The start and command trace are outside this
screenshot; the immediately preceding three-sample baseline was J1 144.93.

Relative to that baseline, reported J1 travel is +4.22 degrees, ending 0.78
degrees below the commanded target. J2-J6 endpoint readings match the baseline,
and the user confirms no physical movement of other joints. Together with the
previous decreasing attempt (-4.13 degrees, 0.87 degrees short), this establishes
two supervised isolated J1 movements in opposite commanded directions, both
ending short of their targets. Neither met the existing 0.25-degree target
tolerance. Two trials do not establish statistical repeatability or calibration.

The cause of the endpoint shortfalls remains unverified; no servo deadband,
friction, encoder error, or tuning diagnosis is established by these results.
No pass-tolerance widening, automatic correction, new joint test, calibration,
or further motor command is authorized merely by this outcome. Keep the recorded
endpoint 149.15 as historical telemetry, not an automatically approved startup
pose. The next plan should address the observed target accuracy before broader
automatic initialization or multi-joint sequences.

## Positioning observation prepared, October 21, 2026

The user requested beginning the next positioning milestone. Prepared
`bucke_arm.check_joint1_position` for one fixed J1 decrease from the latest
recorded 149.15 to 144.15 degrees at speed setting 10. The previous helper is
unchanged and supplies shared telemetry checks and the serial-write guard.

The new helper checks the complete reviewed pose, health, diagnostics, enabled
servos, firmware, SDK, and serial settings before motion; it rechecks pose,
health, and diagnostics after its `CHECK J1` operator prompt. Default mode is a
preview with no SDK import or connection. Hardware mode records timestamped CSV
samples throughout a 20-second observation, even following early target arrival.
The final three samples must be stationary within the existing +/-0.25-degree
tolerance. An unmet target or another monitored failure requests stop, without
retry, correction, return, calibration, or controller-setting changes.

Twenty seconds is an experimental observation duration to investigate additional
settling; synchronous queries can extend elapsed wall time. Existing temperature
uncertainty and software-monitoring limitations remain. No cause of the earlier
shortfall or broader repeatability has been established. All 63 offline tests
passed, including delayed settling, persistent shortfall, late drift, bad starting
state, post-prompt pose changes, lost readings, unexpected movement, interrupted
or partial writes, and logging failure after the command. Hardware execution and
physical confirmation are pending. See the arm README's next positioning check
for copy/run instructions.

## Positioning observation completed and ready-pose intent confirmed

The user supplied `j1_position_20261021T171805_256457Z.csv` and terminal
screenshots. The file contains 87 data rows, including 82 observation samples.
Preflight and immediate pre-command angles were
`[149.15, -14.58, 11.16, 122.78, 86.48, -45.7]`. The terminal shows expected TX
`fe fe 06 21 01 38 4f 0a fa`, host write acceptance of nine bytes, and raw RX
`fe fe 03 21 01 fa`. These are communication evidence, not target-arrival proof.

J1 first reported 145.10 at elapsed 0.820 seconds and remained there through
19.978 seconds and the post-stop sample at 20.258 seconds. Its reported travel
was -4.05 degrees toward target 144.15, ending 0.95 degrees short. All recorded
moving/error values were zero; samples did not capture a moving=1 state. J2-J6
readings were unchanged throughout. The terminal confirms the target check
failed, stop was requested with response 1, final servo status was all zero,
and temperatures were `[38, 35, 30, 44, 66, 34]` C. Additional observation did
not resolve the reported shortfall.

A subsequent screenshot of the read-only `is_in_position(target, 0)` check
shows current angles `[145.10, -14.58, 11.16, 122.78, 86.48, -45.7]`, in-position
reply 1, moving 0, and error 0 for queried target
`[144.15, -14.58, 11.16, 122.78, 86.48, -45.7]`. This shows that query accepted
this offset; it does not establish the exact tolerance or explain the shortfall.

The user subsequently confirmed only physical J1 moved during this test and
that initialization means moving into a known ready pose for picking objects.
No automatic ready pose, acceptance-tolerance widening, calibration, or next
joint command has been selected. Review of current physical configuration and
workspace is needed to choose subsequent joint tests and the ready-pose route.

## Relocated base, mat coordinates, and fresh stationary snapshot

The user relocated/reoriented the base and supplied new photos. They report its
center approximately at mat (0,0), with corners approximately (+/-3,+/-3).
Each grid square is one inch (25.4 mm). Pickup remains mat (7,2). The user
confirmed the first coordinate increases toward the front table edge, and the
second toward the right in the supplied front view. Thus the nominal mat-plane
offset is 177.8 mm toward the front and 50.8 mm right, about 184.9 mm horizontally
from the reported base center. These are approximate mat coordinates, not
verified robot-frame targets. Robot-axis alignment, mat-plane height relative
to the robot origin, and gripper-tip reference remain to be established.

The latest `pi_status.py` screenshot reports three identical angle lists
`[144.84, -14.76, 11.16, 122.95, 86.74, -45.79]`, moving 0/error 0 each time,
SDK 4.0.6, firmware 7.3, `/dev/ttyAMA0` at 1000000 baud. Servo status is all zero;
voltages are `[12.0, 12.0, 11.9, 7.9, 6.9, 7.9]` and temperatures
`[38, 35, 30, 46, 69, 34]` C. J5's reported temperature has risen from prior
66/67 C readings; no exact-revision thermal limit or thermal clearance has been
verified. Zero status does not establish thermal clearance. Earlier movement
helper baselines do not apply to this snapshot. No new motion target or change
to calibration/settings was made from these observations. Next preparation is
read-only coordinate/reference inspection, followed by physical frame and tool
reference verification before selecting an approach target.

## Coordinate/reference query and J5 temperature increase

The user's subsequent read-only query screenshot reports angles unchanged at
`[144.84, -14.76, 11.16, 122.95, 86.74, -45.79]` and coordinates
`[36.8, -4.1, 307.4, 87.06, -14.79, -32.81]` (mm/degrees). Reference frame is 0,
end type is 0, and tool/world reference lists are both six zeros. End type 0 is
the flange according to SDK 4.0.6, so these coordinates are not calibrated
gripper-tip coordinates. Reference frame 0 is base; this does not establish
alignment of robot X/Y with the mat axes or mat-plane Z.

Temperatures are now `[38, 35, 30, 46, 70, 34]` C, moving 0/error 0. J5 has
continued rising during the stationary read-only checks. Motion preparation is
on hold pending cooldown and verification of applicable thermal guidance; 70 C
is not asserted to be a verified fault threshold for this unit. Do not infer
thermal clearance from error 0. No servo release, power, reference-setting,
calibration, or movement command was included in the supplied query. Remaining
ready-pose work includes physical mat/base alignment and gripper-tip offset/
mat-height measurement; a zero tool-reference list does not supply those data.

## Gripper measurements, cooldown, and second J4 recovery

The user detached the gripper and supplied eight measurement photos. Its label
lists size 110x90x60 mm. They report mat-to-gray-base-bottom height about
0.5 inch (12.7 mm), along-gripper mounting-pin-midpoint to gripping-point distance
3 14/16 inches (98.425 mm), and perpendicular mounting-face to pad-center distance
3 15/16 inches (100.0125 mm). The user explicitly confirmed the latter direction.
These remain approximate user measurements, not a verified 3D tool transform;
the photographs do not resolve all mounting axes/endpoints. No tool reference
was written. In particular, the reported perpendicular distance needs geometric
verification before use, given the labeled overall dimensions.

After cooldown with the gripper detached, a new snapshot reports angles
`[108.01, -19.59, 11.07, 153.54, 74.97, -16.96]`, stationary on all three samples,
error 4, zero servo status, temperatures `[32, 31, 27, 32, 32, 29]` C, and voltages
`[12.1, 12.1, 12.1, 7.8, 7.5, 7.6]`. The changes in cooling time, pose, and
gripper loading prevent attributing the temperature reduction to one cause.
Read-only J4 inspection returned limits -150/+150 degrees, enabled 1, free mode
0, seven zero diagnostic entries, and error 4. The prior helper was reviewed;
the user then reported independently using the previous J4 release function
to recover the position. A subsequent screenshot shows three stable readings
`[107.92, -19.59, 11.07, 145.72, 76.2, -16.25]`, moving 0/error 4, zero servo
status, and temperatures `[34, 32, 28, 33, 38, 31]`. When asked about re-engagement,
the user confirmed response 1; no separate physical holding observation was
explicitly supplied in that answer.

The user subsequently ran the guarded error-clear snippet. Its screenshot shows
zero servo status, seven zero diagnostic entries, temperatures
`[34, 33, 28, 37, 46, 33]`, and three stationary pre-clear samples at
`[107.92, -19.59, 11.07, 145.72, 76.28, -16.25]` with error 4. One SDK clear call
returned 1, followed by three identical stationary samples with error 0 and a
printed PASS. This confirms error clearing and stable reported pose during
that observation, not calibration or physical collision clearance. Next is
read-only evaluation of hypothetical small J2 changes before proposing any
supervised movement. No J2 target or automatic initialization is approved by
these observations alone.

## Standby step prepared from offline model and updated side views

After the user redirected the priority to reduced-load waiting, the prior
standalone J2 test proposal was withdrawn. The manufacturer's 280 Pi URDF was
evaluated offline. It reproduces the latest reported flange XYZ within about
1.38 mm for one pose, but contains no inertial elements. Neither real motor
loads nor collision geometry nor thermal behavior was validated. See
[standby_plan.md](standby_plan.md) for candidate geometry and calculation limits.

The user's three subsequent side views confirm the gripper is detached and show
the wrist and a loose cable near the upper link. J4 holding without support was
explicitly confirmed by the user. One fixed J5 76.28 -> 71.28 degree test at
speed 10 is prepared in `bucke_arm.test_joint5`, with preview default, one-write
guard, reviewed-pose and controller checks, stop-on-failure monitoring, no
automatic continuation, and an experimental gate at the last reviewed J5 49 C
reading. This gate is not a manufacturer rating. All 75 offline tests passed.
Authorization, in-person cable/wrist clearance checks, actual SDK packet capture
on the Pi, and physical execution remain pending. No new movement has been run
by the assistant.

## User-run J5 command and consolidated next-stage preparation

The user supplied a photo of `~/test2.py` containing
`mc.send_angle(5, 71.28, 10)` followed immediately by `get_angles()`. Terminal
photos show several invocations and one output angle list
`[108.10, -19.59, 11.07, 146.42, 72.33, -16.78]`. The number of successful movement
transmissions and timing are not established. Relative to the prior snapshot,
J5 changed -3.95 degrees, J4 +0.61, and J6 -0.53; J5 was 1.05 degrees short in
that sample. This is not confirmed settled telemetry or physical isolation.
The assistant's earlier helper was not used and its starting pose is obsolete.

The user requested a much faster workflow, reusing known setup information and
combining preflight, agreed movement, settling, and final angles/error/temperature
into one run. They then requested moving on. `pi_standby_step.py` is prepared
for one fixed J5 target 67.33 degrees, speed 10, from the newly reported endpoint.
It checks the unsettled result before offering motion and requires local
`MOVE J5` confirmation that only J5 physically moved previously and clearance/
mounting/gripper conditions remain satisfied. Other visible joint movement or
uncertain isolation requires cancellation. No extra chat approval is requested.

The stage retains reviewed-pose, fault, enable, diagnostic, packet, thermal-gate,
interval and other-joint-drift checks. It reports settled observation separately
from target accuracy, without widening the +/-0.25-degree criterion or attempting
correction. At least one degree of reported J5 decrease is needed to count a
movement/settling observation. Exact mechanical accuracy, physical isolation,
thermal clearance, and the remainder of the standby route remain unverified.
All 84 offline tests passed. Execution of the new combined stage is pending.

## Combined J5 stage result and locally supervised wrist-phase preparation

The user supplied a screenshot of the combined stage. Preflight angles were
`[108.10, -19.59, 11.07, 146.42, 72.24, -16.78]`, temperatures
`[37, 34, 30, 38, 48, 34]` C. The user typed `MOVE J5` after the displayed prompt
requiring prior physical isolation and clearance. TX was
`fe fe 06 21 05 1a 4d 0a fa`, raw RX `fe fe 03 21 01 fa`. The reported settled
endpoint was `[108.10, -19.59, 11.07, 146.42, 68.37, -16.78]`, moving 0/error 0,
temperatures `[37, 34, 30, 38, 48, 33]`. J5 travel was -3.87 degrees; target
67.33 was missed by +1.04. Other reported joint endpoints were unchanged.
The output explicitly reports failure of the +/-0.25-degree accuracy check.
It is evidence of a stationary reported endpoint, not physical isolation,
calibrated accuracy, temperature safety, or full standby completion.

To reduce repeated transfers and chat turns, `pi_standby_step.py` now offers
`--toward-zero`, with fixed small J5 targets through zero, and a fresh local
`MOVE J5` authorization plus state/clearance checks before every movement.
The operator must cancel if the prior move was not physically isolated, the
next wrist/plate/cable sweep is uncertain, or continuing is otherwise unsuitable.
No other joint is commanded. The phase rejects cumulative other-joint drift,
retains the experimental 49 C gate, and stops offering further targets if
the settled endpoint error exceeds an explicit experimental 1.25-degree
progression gate. That gate is distinct from the unchanged +/-0.25-degree
accuracy test. No actual torque or collision-clearance certification is made.
All 92 offline tests passed, including the full exact target list, one
confirmation per movement, cancellation, new faults between steps, progression
shortfall rejection, fixed other-joint references, and cumulative drift abort.
The guided phase has not been executed on hardware. See the README for details.

## Assistant-selected temperature gate removed from active standby script

The user's guided-phase screenshot shows checkpoint 1 refusing on the 49 C gate
before a before-state summary, confirmation, or movement trace. No movement
from that invocation is evidenced. The actual temperature is not printed in
that failure output and must not be inferred. The user explicitly objected to
the gate and requested completing the load-reduction task.

The assistant acknowledged that 49 C was an assistant-selected conservative
gate, not a manufacturer limit. It was removed from active `pi_standby_step.py`.
Temperatures remain validated and reported; nonzero controller/servo faults,
invalid telemetry, unexpected motion, overshoot, excessive drift, and bounded
settling checks remain in force. The fixed target plan, exact-write checks,
per-step local prompts and progression/accuracy distinction are unchanged.
All 94 offline tests passed, including valid readings above the old gate and
continued stop-on-servo-fault behavior. Updated hardware execution is pending.

## Remaining unknowns

Hardware revision; exact gripper model and connection; independent mounting and power verification; tool offset; orientation convention; measured joint limits; validated starting and transport poses; camera calibration; docking tolerances; and safe motion behavior remain unverified. Firmware is observed as 7.3; manual J4 recovery and error clearing are confirmed by the user and screenshots. The first powered target-position test failed due to unexpected changes in other joints; the cause remains unresolved.

## Evidence

The four original text outputs are preserved under [evidence/2026-10-13](evidence/2026-10-13):

- `buck-e-pi-info.txt`: OS, kernel, Python, and installed packages.
- `buck-e-pi-info2.txt`: SDK location, constants, and method signatures.
- `buck-e-pi-info3.txt`: serial devices and user groups.
- `buck-e-pi-info4.txt`: installed arm constructor source.

Parent-constructor inspection, telemetry results, and deployment errors above were transcribed from screenshots provided in this chat. Their original text logs were not supplied. [hardware.observed.json](../config/hardware.observed.json) summarizes the observations for reference; the current adapter does not load that file.
