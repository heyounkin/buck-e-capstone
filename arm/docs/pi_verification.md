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


## November 18 continuous pickup preparation

The operator specified a one-inch-tall block, slightly over one inch wide,
at mat (7, 2), and measured the lowest finger point 1/2 inch below the pad
midpoint with the gripper pointing down. They subsequently confirmed physical
clearance. The planned midpoint height is 5/8 inch, leaving 1/8 inch nominal
finger/table clearance. This supersedes the earlier 1/2-inch candidate, which
would have zero nominal tip clearance.

The local pickup command now supports one measured approach from saved ready
and jaw closure after joint/status and FK endpoint checks. It requires jaws
already open. No correction trims, lift, return, sweep or drop are issued.
Closure is a command rather than grasp confirmation. The operator's clearance
confirmation does not imply an assistant-executed traversal; hardware execution
of the new pickup sequence has not been observed. See `resume_hover.md` for
execution and recording details.


## Pickup low approach and late rotation reported

The operator reported that pickup went too low and that the gripper continued
rotating afterward. Three stationary, error-free samples were identical:
[36.47, -21.53, -117.15, 43.94, -0.08, 160.22] degrees. The provisional pad
reporter estimates (6.7917, 1.7286, 0.4413) inches at this snapshot, below even
the model's nominal 1/2-inch finger-extension allowance. J4 is 1.7485 degrees
below the planned target, outside the 1.5-degree completion tolerance. These
samples alone do not show the earlier path, contact or whether closure occurred.

The commanded J6 endpoint +161.63 differs from saved ready -135 by +296.63
signed degrees. A single send_angles command does not establish simultaneous
joint arrival or a fixed Cartesian gripper orientation. The pickup solver held
J5/J6 and an arm-angle sum relative to old HOVER; it did not plan constant tool
orientation throughout the approach. Do not wrap or substitute the J6 target
without checking the tool offset and route.

Local hardware pickup entry points are disabled before any queries or motion.
The Pi remains unchanged until those files are transferred. No new target,
return command, tolerance change or calibration correction was selected.


### Operator confirmation of contact and actual XY

The operator subsequently confirmed that the fingers touched the table, the
jaws did not close, and the gripper was centered at mat (6, 2) inches. This is
the observed contact outcome of the new pickup attempt; it supersedes the
pre-execution status in the preparation section above. Compare this reported
physical XY with the latest stationary-angle FK estimate (6.7917, 1.7286):
predicted-minus-observed XY is approximately (+0.7917, -0.2714) inches,
or (+20.11, -6.89) mm. This comparison assumes the physical observation refers
to the same gripper midpoint and unchanged stopped configuration. It is not a
full XYZ calibration sample: pad midpoint height and wrist flange center were
not measured. Table contact must not be converted into an exact pad height
using the approximate finger-extension measurement or modeled tilt.

No jaw closure is consistent with the failed completion checks, but the
pickup run output/trace is needed to establish the actual failure reason and
joint timing. Table contact may have affected the reached angles; neither
controller error 0 nor the stopped readback establishes an unobstructed route.
Do not compensate with a blind +1-inch X shift or a global Z offset. The next
calibration evidence should pair stationary angles with physical mat XYZ of
the pad midpoint and actual robot flange center, without another pickup move.
Pickup remains disabled locally. No recovery motion was commanded.


### Measured pickup trace confirms staggered joint arrival

The operator supplied the saved forward capture, preserved at
[evidence/2026-11-18/pickup-forward-20261119T020112-981571Z.json](evidence/2026-11-18/pickup-forward-20261119T020112-981571Z.json).
The UTC filename corresponds to the November 18 local session. The terminal
prompt after the pasted JSON was excluded without altering the JSON data.
The trace contains 169 samples over 22.069 seconds, maximum gap 0.183 seconds,
and ends incomplete with failure `KeyboardInterrupt`.

J1 through J5 reach within 0.5 degrees of their final recorded values and stay
there by 4.940 seconds (individual times: J1 4.940, J2 0.937, J3 4.565,
J4 1.850, J5 4.181 seconds). J6 is still at -6.94 degrees at 4.940 seconds,
then advances to +160.22 by 11.274 seconds. Its actual recorded change is
294.16 degrees from the initial -133.94. First stationary status after motion
is at 11.429 seconds. Thus late wrist rotation is measured, not merely inferred
from endpoint angles. At nominally the same speed setting, unequal joint
travels did not result in coordinated joint arrival.

The provisional closed-pad reporter estimates Z 0.496 inch at 4.940 seconds
and a minimum sampled Z 0.4294 inch at 11.274 seconds; these are model estimates,
not measured physical heights or a timestamp of table contact. The rotating
off-axis pad reference also changes modeled XY while J6 continues rotating.
The trace cannot establish when physical contact occurred or whether contact
caused the J4 shortfall. Controller error remains zero in the captured samples.

The capture continued stationary until Ctrl+C at about 22 seconds. J4's final
1.7485-degree shortfall exceeds the 1.5-degree target check, so the approach
never qualified for jaw closure; the polling loop had a 120-second deadline.
This is consistent with the reported open jaws. One endpoint command was not
a coordinated Cartesian approach. Correcting this requires both a validated
pad/frame model near pickup and a reviewed trajectory coordinating orientation
with descent; changing target offsets or wrapping J6 alone is not justified.
No additional hardware motion was run and pickup execution remains disabled.


### Operator reports unusual J2 give at stopped pickup pose

The operator reports J2 resting at -21.26 degrees and being easily moved by
hand to a reported -20.74 degrees, unlike the other joints. This is a
0.52-degree reported-angle change. Power/holding state during this observation,
release behavior, load and physical displacement were not supplied; no cause
or hardware adjustment is established. Earlier stationary pickup J2 was
-21.53, so this is a subsequent observation, not the same angle snapshot.

Holding the other five joints at the last supplied stopped values
[36.47, _, -117.15, 43.94, -0.08, 160.22], current provisional FK predicts
that changing J2 from -21.26 to -20.74 moves the midpoint approximately
(+1.056, +0.705, +1.570) mm, including +0.06182 inch in Z. This sensitivity
is conditional on those other angles and model geometry, not a measured
physical displacement. The Z change is about half the intended 1/8-inch
nominal table margin. It does not explain the approximately one-inch observed
X miss or the recorded staggered J6 arrival by itself. Pickup stays disabled;
no torque, calibration, servo-release or motion settings were changed.


### J2 follow-up: bounded give with servos engaged and table contact

The operator confirmed the arm is powered with servos engaged. J2 remains at
-21.26 degrees when untouched, can be lifted by hand to approximately -20.74,
and returns to -21.26 after release. They describe boundaries beyond which
it does not move without a command. This does not establish uncontrolled sag
or inability to return to the commanded position. The operator also confirmed
that the fingers are currently touching the table. Contact can influence the
resting configuration, so this sample is not a free-space commanded-versus-
settled calibration measurement.

The requested next task is J2 characterization. Establish a fresh full-angle,
stationary, error-free baseline before choosing a single J2-only lift. Measure
commanded-versus-settled response after table contact is removed, without hand
pressure or wrist/gripper commands. No automatic pickup, compensation, joint
parameter adjustment, or recovery motion has been enabled or executed.


### J2 command-response observations and manual diagnostic reporting

The operator commanded J2 +2.00 from -19.68 at speed 10; reported J2 stopped
at +0.61 (reported-minus-requested error -1.39 degrees). A subsequent command
from +0.61 to -19.00 at speed 10 stopped at -19.59 and remained there over
repeated P reads (error -0.59). The local +0.60 compensation trial commanded
-18.40 from -19.59, also at speed 10, but neither visible movement nor reported
angle change occurred. Subsequent P still read -19.59, a -1.19-degree error.
No constant compensation or specific deadband threshold is established.

Inspection found the generic move_joint wait uses the existing 3-degree pose
tolerance, so an unchanged position 1.19 degrees away can satisfy its three
stationary-read completion check. This explains why the application returned
to the menu, not why the actuator did not move. Do not attribute the physical
nonresponse to that completion check without more evidence.

The manual controller now prints requested/reported angles, signed error,
reported change and the existing tolerance, and explicitly flags no reported
movement. It sends no additional motion or correction. Q at the desired-angle
prompt now cancels without trying to parse a number. Motor settings, target
tolerances and hardware command behavior were not changed. Pickup remains
disabled locally; no further J2 movement was commanded by the assistant.


### J2 larger upward request reaches only -19.16

From -19.59 the operator commanded J2 -17.50 at speed 10. Reported J2 moved
+0.43 degrees to -19.16 and stayed there on the subsequent P read. The final
reported-minus-requested error is -1.66 degrees; the commanded change was
+2.09 degrees. Other reported joints remained [36.65, _, -117.59, 44.47,
0.08, 160.75]. This is additional evidence against adopting a constant +0.5
or +0.6 correction. It does not establish an exact servo deadband, weak motor,
mechanical fault or firmware cause.

The next diagnostic is read-only comparison of raw servo control registers.
The manufacturer pymycobot v4.0.6 demo reads addresses 7, 21, 22, 23, 24, 26,
27 via get_servo_data. Its write and power-on sections must not be invoked for
this diagnostic. Reference:
https://github.com/elephantrobotics/pymycobot/blob/v4.0.6/demo/pid_read_write.py
and the read API implementation:
https://github.com/elephantrobotics/pymycobot/blob/v4.0.6/pymycobot/generate.py
Raw values from other joints are comparison data, not approved J2 defaults.
No servo settings or motion behavior were changed.


### Read-only control parameter capture

The operator supplied raw addresses [7,21,22,23,24,26,27] with these values:
J1 [0,15,20,0,0,3,3]; J2 [0,32,8,0,0,3,3];
J3 [0,32,8,0,0,3,3]; J4/J5/J6 [0,25,25,0,0,3,3].
Initial and final reported angles were identical:
[36.65,-19.16,-117.59,44.47,0.08,160.75].
J2 and J3 match, including the manufacturer's example parameter array; this
is not proof of correct factory values, adequate holding torque, or absence
of a mechanical issue. The raw 3 values must not be interpreted as degrees.
No register writes, power changes, or motion commands were made.

SDK v4.0.6 exposes read-only get_angles_plan, get_fresh_mode,
is_servo_enable and get_encoder queries. The next diagnostic compares their
raw replies with actual angles. The SDK describes get_angles_plan only as
angle-plan data; its exact relationship to the last commanded endpoint on
firmware 7.3 must not be assumed. The parameter documentation located for
another 280 variant contains inconsistent integral/derivative descriptions,
so it does not justify tuning this unit's gains. No gains were changed.


### Controller angle plan matches the J2 request, but actual J2 remains short

The operator's next read-only capture returned actual angles
[36.65,-19.16,-117.59,44.47,0.08,160.75], angle-plan reply
[37.28,-17.5,-117.11,45.68,0.08,161.63], refresh mode 1, J2 enabled 1,
and raw J2 encoder 2266. The angle-plan J2 entry matches the -17.50 request,
while reported actual J2 remains 1.66 degrees below it. This supports receipt
of the target by the controller; it does not establish the cause of poor
tracking. No raw-encoder output-angle conversion or servo-register tuning
was assumed. See [consolidated J2 diagnostic report](j2_diagnostic_report.md)
for the command-response table, raw settings and model-specific service
questions. No further motion or parameter changes were made.


### Subsequent HOVER sample: measured height and ambiguous pointing direction

The operator ran the original continuous HOVER from actual ready
[-89.12,1.4,-0.52,-0.79,89.2,-133.5]. The recorded final angle snapshot was
[33.92,-13.35,-93.95,12.83,0.52,160.48]. The run reported completion within
joint tolerance and saved forward-20261119T030643-478818Z.json on the Pi;
that trace has not been supplied locally. The nominal pad reporter estimates
(7.315,1.769,2.359) inches at these actual angles. The operator had described
the physical pad center as slightly over 2.25 inches high and subsequently
said the angled gripper points toward (6.5,2). That pointing coordinate is
not established as the vertical projection of the gripping-pad midpoint.
Do not use it as a physical XY calibration sample or add a +0.5-inch correction.

Current nominal flange +Z is about 4.50 degrees from downward at this angle
snapshot (a flange-model result, not a measured gripping-finger orientation).
Joint tolerance does not verify pad XYZ or orientation. The next local target
requires an unambiguous physical pad-midpoint XY observation; wrist orientation
and translational alignment must not be conflated. No new hardware movement
or target adjustment was issued for this observation.


### Physical hover midpoint confirmed over (7,2); local orientation test prepared

The operator clarified that the pad midpoint is vertically above mat (7,2),
while the gripper is tilted toward (6.5,2). This supersedes treating the aiming
point as physical midpoint XY. With actual hover angles
[33.92,-13.35,-93.95,12.83,0.52,160.48] and midpoint slightly above 2.25
inches, XY is already physically aligned. The model's XY (7.315,1.769) is not
a reason to shift the observed physical midpoint toward model (7,2).

A separate local high-hover orientation candidate preserves that snapshot's
MODELED midpoint, holds signed J6 at 160.48, and solves for downward nominal
flange +Z. Candidate: [34.3843,-7.6479,-104.5846,22.2325,0,160.48]. The
model predicts less than 0.01-degree flange tilt and less than 0.01 mm endpoint
midpoint change. This is an intended local correction, not a guarantee of
physical verticality or preserved physical XYZ given tracking/calibration error.

Independent progress sampled at [0,.25,.5,.75,1] for J1-J5 with J6 fixed gives
minimum nominal pad height about 1.325 inches. The path may dip even though
endpoint height is preserved; remove the block and clear the local sweep.
This sampling is not continuous collision verification. No new hardware run
has been observed. Existing low pickup and return stay disabled.

`python3 -m bucke_arm.hover_alignment` previews the candidate. `--execute`
performs one all-joint endpoint command at speed 5 only from the exact recorded
high-hover neighborhood (0.5-degree tolerance, three stable stationary reads,
live limit checks and start recheck). It records direction `high_hover_alignment`,
requires three stationary samples within 1.5 degrees in 20 seconds, and stops
on failure/interruption. It never closes the jaws or adds corrective commands.
Final output reports actual-angle model tilt and midpoint drift for comparison;
physical midpoint and finger orientation must still be checked before descent.


### Alignment refused before movement; checkpoint refreshed from three samples

The alignment command refused at the starting-pose guard and did not produce
a forward-recording message or issue the alignment move. Readback was
[34.27,-13.35,-93.95,13.0,-0.35,160.66]. The operator then supplied three
identical stationary, error-free samples of those angles, with zero servo
status. Relative to the earlier checkpoint, J5 changed by -0.87 degrees,
exceeding the existing 0.5-degree entry tolerance. This is a between-snapshot
change, not evidence of drift during the latest three stationary samples.

The alignment module now uses that explicitly supplied stable checkpoint and
recalculates the nominal midpoint-preserving vertical-flange candidate:
[33.9587,-8.2415,-103.8620,22.1035,0,160.66]. J6 remains at its new measured
160.66. The 0.5-degree entry tolerance and 0.25-degree starting-stability check
were retained. Offline independent-progress sampling on five levels gives
minimum nominal midpoint height 1.3879 inches; no continuous physical-clearance
guarantee is implied. The block must remain out for the local alignment test.
All 133 tests passed locally, including refusal from the superseded checkpoint.
The hardware alignment itself has still not been observed.


### Physical post-alignment anchor and raised approach trial prepared

The operator observed the post-alignment pad center at (6 5/8,1 3/4) inches
and a little straighter. They initially reported height 3 inches, then
explicitly corrected it to 2 inches. Pair (6.625,1.75,2.0) with the latest
recorded stopped joints [34.18,-9.66,-104.23,20.47,-0.08,160.66].

The requested raised point is (7,2,1.625), so use measured relative motion
(+0.375,+0.25,-0.375) inches rather than another absolute model target.
A one-command local trial with bounded direction-specific feedforward was
prepared as documented in resume_hover.md. This is a trial, not established
J2 calibration. Independent progress sampled at five levels for J1-J5 gives
minimum nominal pad height 1.2506 inches to the precompensated command;
this is not continuous swept-volume verification and the block stays out.
J6 is unchanged, low pickup/return remain locked, and no new hardware run has
been observed. No servo parameters or kinematic calibration were changed.


### Local approach refused before movement; live start within observed region

The operator's local approach was refused before transmission at
[34.27,-9.66,-104.23,21.0,-0.52,159.96]. J4 differs +0.53 degrees and J6
-0.70 from the prior post-alignment sample, so the 0.5-degree fixed gate failed.
No new local approach recording or motion was reported. Rather than rewriting
one snapshot again, the local approach now captures a stable live pose inside
the explicit box spanning these two supplied observations with 0.25-degree
padding. It retains fault/stationarity, 0.25-degree stability, repeated start
validation and live limit checks. The measured physical anchor stays paired
with its original angles; the candidate is recalculated holding live signed J6.

All 142 local tests passed, including acceptance of the newer observed pose,
refusal of unrelated or unstable poses and changes during limit checks, and
verification that live J6 and the fixed measured-relative XYZ goal are preserved.
This is tested software behavior, not a successful physical positioning run.
No further hardware movement or setting change has been observed.


### Local raised approach arrived with close J2/J4 tracking

The operator executed the live-start local approach. Captured start was
[34.27,-9.66,-104.23,21,-0.52,159.96]. Executed command was
[34.2137,-13.3825,-107.9675,33.942,0,159.96]; intended reported angles were
[34.2137,-13.9725,-108.3355,32.308,0,159.96]. Final actual angles:
[34.27,-14.06,-108.36,32.34,-0.52,159.96]. The run reported arrival within
joint tolerance and saved forward-20261119T033411-539397Z.json on the Pi.
The underlying trace has not been supplied locally.

J2 reported-minus-desired error is -0.0875 degrees and J4 error +0.032.
The local anchored midpoint estimate is (7.023,1.953,1.609) inches and nominal
flange tilt 0.526 degrees. This supports the bounded feedforward trial at this
specific configuration/direction, not general correction values for all poses.
Actual physical endpoint, finger orientation and block clearance still need
confirmation. No jaws or return were commanded. The target pad height is above
the one-inch block, so this is a raised approach and not a grasp. A structured
observation is saved in config/raised_approach.observed.json; it is not loaded
automatically as a route. No new descent or ready-to-approach path was enabled.


### Physical raised approach confirmed; short descent/closure step prepared

The operator confirmed that the jaws are centered over the block and almost
straight down, with the lowest fingertip overlapping the block slightly.
This physically confirms the local approach's alignment, not its printed
absolute height. No exact pad height was measured at this endpoint. Actual
joints were [34.27,-14.06,-108.36,32.34,-0.52,159.96].

A separate grasp step now plans a relative 0.25-inch descent from a stable
live pose near that physically verified configuration. It preserves current
nominal XY, bend, J5 and J6. Reference desired angles are
[34.27,-16.031,-110.2681,36.2191,-0.52,159.96]; the same bounded directional
trial offsets yield command
[34.27,-15.441,-109.9001,37.8531,-0.52,159.96]. Independent joint progress
sampled at five levels predicts at most 0.3417-inch pad lowering toward that
command, not a physical continuous-clearance guarantee.

The step sends one command, records actual motion, then requires stationary
fault-free arrival and relative FK checks (XY drift <=0.125 inch, lowering
0.125-0.375 inch, nominal flange tilt <=2 degrees) before closure. It commands
closure only, with no sensed grasp confirmation, lift, return or sweep. The
old ready-to-low-pickup entry point remains disabled. Hardware execution of
this new short descent/closure step has not yet been observed.


### Local grasp and J2 lift physically confirmed

The operator executed `grasp --execute`. Captured start was
[34.18,-13.97,-108.36,32.95,-0.52,160.22]; final actual joints after the
quarter-inch descent were [34.27,-15.90,-110.39,36.73,-0.35,160.22].
The program reported descent checks passed and jaw closure commanded, saving
forward-20261119T034038-546219Z.json on the Pi. The operator supplied two
photos and clarified that the closed-jaw photo is the after image. A secure
lift was not inferred from photos alone.

With jaws kept closed, the operator made two J2-only manual lift commands at
speed 5. Command -12.00 from -15.90 reached -13.62, with final pose
[34.10,-13.62,-110.39,37.17,-0.52,160.04]. The block stayed in the jaws,
but one corner was barely touching or clearance was uncertain. Command -10.00
then reached -11.60, with three-dimensional configuration
[34.18,-11.60,-110.39,37.17,-0.61,160.04], repeated unchanged by P. The
operator explicitly confirmed the block was fully off the table. This is a
physically demonstrated local grasp/lift at the pickup location.

The final requested-minus-reported J2 difference is 1.60 degrees; actual -11.60
is the observed held pose, not commanded -10.00. Printed pad estimate
(7.85,1.99,2.05) inches is nominal FK output, not measured physical XYZ.
The record is saved in config/pickup_sequence.observed.json. No sweep/drop has
been performed. The two observed lift steps have not yet been combined and
executed as one lift. Direct ready-to-grasp motion and automatic return remain
unverified/disabled; this success must not unlock the earlier contact route.


### J1 sweep and landing physically confirmed

At speed 5 with the block lifted, the operator commanded J1 70,75,90,91.
Actual J1 endpoints were 69.25,74.17,89.03,90.08. Last pose, repeated by P:
[90.08,-11.60,-110.39,37.17,-1.14,160.04]. The operator reported the block
landed about mat (2.5,7), rather than the initially requested (2,7). Landing
is the operator's physical observation; the supplied transcript does not show
the O release command. Nominal XYZ (2.59,7.49,2.05) is not the measured landing.
The observed pickup/lift/sweep/drop is saved in pickup_sequence.observed.json.

A combined `pick_and_place` routine is prepared from the physically verified
raised pickup pose only. It opens jaws, runs the existing guarded quarter-inch
descent and closure, waits for valid gripper-stopped replies with a two-second
minimum, then commands one J2 -10 lift and one J1 91 sweep before release.
The single joint intervals are the same monotonic intervals covered by the
manual steps, but this combined sequence has not been hardware-tested.
Gripper polling is supported by SDK 4.0.6 but has not yet been observed on this
unit. It aborts on invalid replies or timeout rather than lifting blindly.

All future target limits are preflighted before any command. Lift must reach
J2 within 0.4 degrees of the observed -11.6 and show nominal rise >=0.5 inch
before sweep. Drop checks J1 near observed 90.08 and nominal height retention
before release. Failure requests stop without automatic return or retry. The
ready/drop pose is rejected before opening jaws. No direct ready-to-pickup,
low contact route or automatic recovery is enabled by this routine.


### Staged return to saved READY prepared at the operator's request

The operator requested return to the saved READY pose and suggested rotating
J6 to -135 before the remaining reset. The target was explicitly clarified
as READY [-90,0,0,0,90,-135], not the raised pickup pose. A signed J6 move
from the recorded drop +160.04 to -135 is -295.04 degrees.

Offline screening found that J6-first then a direct all-joint ready reset can
place the provisional tool envelope 23.4 mm below the mat if J4 resets before
J3 rises. A staged proposal instead raises J2 to 0, sets J6 -135, raises J3
to 0, resets J4 to 0, sets J5 90, then turns J1 to -90. Nominal sampled
tool/table heights remain positive as documented in resume_hover.md, but the
provisional cylinder does not model arm self-contact and actual full swept
clearance has not been established.

The new `return_ready --execute` is a supervised initial traversal: confirmed
empty/open jaws and local Enter confirmation before every single-joint stage.
It retains whole-route limits, stable start/checkpoint checks, motion/error
polling, actual trajectory recording, cancellation and stop-on-failure.
No automatic retry or return from an arbitrary pose is enabled. The old direct
hover return stays disabled. `pick_and_place --return-ready` offers this same
supervised return after release. No hardware execution of the new return has
been observed; software tests and simulation are distinct from physical proof.


### Operator confirmed staged return; J6 speed increased by request

The operator reported the new staged drop-to-READY return worked perfectly.
No return telemetry or trace files were pasted, so this is physical completion
confirmation without a new exact final-angle snapshot. The observed return
used speed 5 for all stages. The operator requested only J6 to -135 be changed
to speed 10. The route, other stage speeds, confirmation pauses, limits,
starting-state, completion and stop checks remain unchanged.

Return J6 now uses speed 10; J2/J3/J4/J5/J1 remain 5. Preview and stage prompts
show the actual per-stage speeds. Recorded trajectory JSON stores the actual
commanded speed, including 10 for return stage 2. The faster J6 variant has
not yet been observed on hardware; its authorization comes from this explicit
operator request. The old direct hover return remains disabled.


### Full READY-to-READY entry point prepared

After confirming the return and requesting J6 speed 10, the operator requested
the whole sequence from the top. `full_cycle` now composes the demonstrated
high HOVER, taught transfer, measured raised approach, grasp/lift/sweep/drop,
and supervised return. This full combined hardware run has not been observed.
Block placement is deferred until the raised approach, reproducing the observed
setup rather than passing the initial high sweep over an untested block.

The taught transfer uses observed command
[33.9587,-8.2415,-103.862,22.1035,0,live_J6] and expected settled checkpoint
[34.18,-9.66,-104.23,20.47,-0.08,live_J6]. The actual live checkpoint is
validated before the existing measured-reference local approach. The original
standalone guards remain; this pipeline owns its prior phase's validated pose
instead of failing on an exact stale hardcoded snapshot. It does not retry or
introduce an unverified direct low READY-to-pickup route. Local placement and
staged return confirmations remain. All future reference limits are preflighted
before the initial open command. No hardware was moved by the assistant.


### First full cycle reached raised approach; signed J5 guard fixed

The operator ran full_cycle from READY. High HOVER reached
[34.01,-13.35,-93.95,12.91,0.79,160.4], the taught checkpoint reached
[34.18,-9.66,-104.23,20.47,0.7,160.4], and the local raised approach reached
[34.18,-14.06,-108.36,32.34,0.7,160.4]. It then refused at the grasp-start
gate before block-placement confirmation, descent or closure. Three subsequent
pi_status samples were identical, stationary, error-free and all servo status
entries zero. Pi traces are forward-20261119T041451-153072Z.json,
forward-20261119T041453-531052Z.json and forward-20261119T041456-000397Z.json;
the files themselves have not been supplied locally.

The fixed angular comparison rejected J5 +0.70 because it differed 1.22 degrees
from the historical -0.52 sample. At the new actual pose, nominal flange tilt
is 0.705 degrees and nominal midpoint change from the verified raised reference
is (-0.0652,+0.1063,+0.0090) inches. Both satisfy the existing 2-degree tilt
and 0.15-inch per-axis position gates. The grasp gate now bounds other joints
as before and evaluates J5 using those unchanged geometric conditions, rather
than a signed difference from one negative reading. No motion/arrival tolerance
or descent distance was changed.

The empty-gripper drop entry also now uses absolute near-zero J5 <=2 degrees
rather than its old signed difference from -1.14. Other observed drop bounds
are unchanged. This prevents the same valid positive-J5 condition from causing
a later return refusal. New tests replay the supplied positive-J5 observations
through the full mocked cycle, retaining wrong-pose, excessive-tilt, shift,
fault and cancellation rejection. All 175 tests pass locally. The remainder
can resume with pick_and_place --execute --return-ready from the present raised
pose after physical block placement/alignment confirmation; no return to READY
or repeated initial approach is needed. A complete hardware cycle is still
unconfirmed, and no new motion was run by the assistant.


### Subsequent full run closed on block; gripper-wait timeout

The operator's next full run reached raised pose
[34.1,-13.97,-108.45,32.34,0.61,160.57], accepted block placement, ran the
quarter-inch descent, and then stopped with "Gripper did not settle" before
any lift. The grasp trace on the Pi is forward-20261119T042619-908090Z.json.
A subsequent read-only probe returned actual pose
[33.92,-16.08,-110.12,36.56,-0.96,160.57] and five identical adaptive-gripper
readings moving=0, value=54. This proves stable partial position after stopping,
not the exact motion-flag behavior during the failed wait. Stop handling may
have changed the flag. No further motion was issued during the probe.

Loaded closure waiting now reads adaptive gripper position as well as motion
status. After at least two seconds it accepts stopped replies, or a valid
partial position (0<value<95) stable within one unit for at least one second.
Opening retains the stopped-flag requirement. Invalid replies, persistently
open closure, changing position, faults or timeout abort with the last raw
moving/value fields. Stable contact is not object identity or force sensing.
No torque, calibration, power or actuator settings changed.

`pick_and_place --execute --resume-grasp --return-ready` now offers an explicit
held-block resume. It requires the known stable post-grasp pose and local
confirmation that the block remains between the jaws. It preflights lift and
sweep targets, checks closure telemetry, then runs the existing lift/sweep/drop
and staged return. There is no initial open, new closure or descent; this
prevents a restart from dropping the already-held block. Ready and drop starts
refuse before movement. The new resume and loaded wait have software tests,
but their physical execution has not yet been observed. All 181 tests passed.

The preceding subdegree J3 handoff refusal was also corrected locally: approach
arrival and grasp/drop entry use shared 1.5-degree joint bounds, while the
existing 0.15-inch midpoint gate and 2-degree near-vertical gate remain. The
midpoint reference is the same measured-anchor goal as the approach, rather
than one incidental historical actual-angle point. New exact-pose full-cycle
tests cover both of the operator's latest observations and the later return.


### Operator confirmed resumed sequence; return prompts removed, J5/J1 paired

The operator reported "Everything looked good" after the held-grasp resume,
then requested that return need no confirmation and J5/J1 be synchronous.
No new terminal telemetry was supplied with this physical success report.
This supports the resumed lift/drop/return completion; it is not a transcript
of one uninterrupted READY-to-READY run or proof of the new paired finish.

Default return now has five stages: J2->0, J6->-135 at speed 10, J3->0,
J4->0, then one all-joint READY command setting J5=90 and J1=-90 together at
speed 5. The other four setpoints remain their existing READY goals. Both axes
are issued in one command; exact simultaneous arrival is not claimed. All
preflight/start/limit/status/completion checks and trajectory recording remain.
Optional callback/--step is diagnostic only; no default return input is requested.
Full-cycle block placement still requires the existing local physical check.

New tests verify no default return prompt, exactly one paired final command,
retained speed profile, correct setpoints despite prior following error,
preflight rejection of paired targets, and stop on paired-stage fault. All 186
tests pass locally; Python 3.8 syntax and fake full cycle also pass. The paired
return itself has not yet been physically observed. No hardware was moved by
the assistant, and the earlier direct-return lockout remains.


### Mid-cycle block placement removed by explicit request

The operator reported the latest cycle worked, then requested not having to
place the block during execution. The full-cycle CLI now selects the
pre-positioned-block callback by default and says to put the block at (7,2)
before starting. It continues only after the existing raised-pose checks;
no motion target, trajectory, controller parameter or completion gate changed.
Return is still automatic with J5/J1 paired and J6 speed 10.

The old placement prompt remains only under --place-at-pickup. New tests prove
the default has no operator input through the cycle, preserve the optional
manual callback, and retain all earlier fault/pose/cancellation tests. All 188
tests pass locally. The first approach past a pre-positioned block has not yet
been physically observed; the prior approach was block-out. No new hardware
motion was executed by the assistant. This does not provide feeding, repeated
cycles or object-presence sensing.
