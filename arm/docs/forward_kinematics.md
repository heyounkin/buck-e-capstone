# Offline forward kinematics

`bucke_arm.kinematics.forward_kinematics(angles_deg)` converts J1-J6 degrees
to a nominal base-to-flange 4x4 transform. It returns XYZ in millimetres,
a rotation matrix, and roll/pitch/yaw in degrees. It uses Python's standard
library and works on Python 3.8 without loading pymycobot.

## Geometry and conventions

Geometry comes from Elephant Robotics' [myCobot 280 Pi URDF at pinned revision
c0d6fbe8](https://github.com/elephantrobotics/mycobot_ros/blob/c0d6fbe8f7d9b18b46a9fcb0b041e5bc2d66196b/mycobot_description/urdf/mycobot_280_pi/mycobot_280_pi.urdf),
retrieved October 28, 2026. The fixed g_base-to-joint1 transform is identity.
The six revolute origins are stored in `kinematics.py` in chain order:

| SDK joint | URDF joint | Origin XYZ (mm) | Origin RPY (rad) |
| --- | --- | --- | --- |
| J1 | joint2_to_joint1 | 0, 0, 139.56 | 0, 0, 0 |
| J2 | joint3_to_joint2 | 0, 0, -1 | 0, 1.5708, -1.5708 |
| J3 | joint4_to_joint3 | -110.4, 0, 0 | 0, 0, 0 |
| J4 | joint5_to_joint4 | -96, 0, 64.62 | 0, 0, -1.5708 |
| J5 | joint6_to_joint5 | 0, -73.18, -1 | 1.5708, -1.5708, 0 |
| J6 | joint6output_to_joint6 | 0, 45.6, 0 | -1.5708, 0, 0 |

Each step multiplies the parent transform by Translation(origin),
Rz(yaw) Ry(pitch) Rx(roll), then Rz(joint angle). All axes are local positive Z.
Manufacturer rounding of origin angles is retained. No fitted joint offsets
or base correction are applied. The SDK-angle mapping is supported
provisionally by historical position comparisons below.

The transform maps flange-frame points into base coordinates; rotation columns
give the flange axes expressed in the base frame. Model RPY uses
Rz(yaw) Ry(pitch) Rx(roll), pitch in [-90, 90] degrees, with yaw chosen as zero
at gimbal lock. Use the rotation matrix to avoid Euler-angle ambiguity.
SDK orientation conventions remain unverified, so comparisons use XYZ only.

## Use

From the `arm` folder, reproduce the earlier standby-planning comparison:

```sh
python -m bucke_arm fk --angles 108.01 -19.59 11.07 145.81 76.28 -16.25 --reported-xyz 20.5 -1.3 290.5
```

Omit `--reported-xyz` for a prediction alone. JSON output includes the full
transform, rotation matrix, RPY, source URL and optional predicted-minus-reported
XYZ differences and Euclidean position error. No pass/fail accuracy threshold
is imposed. The command never constructs RealArm or connects to the robot.

```python
from bucke_arm.kinematics import forward_kinematics, compare_xyz

pose = forward_kinematics([90, 0, 0, 0, 90, -135])
print(pose.xyz_mm)
print(pose.rotation_matrix)
```

FakeArm status now uses this model in place of fixed sample coordinates. Its
legacy `tool_pose_mm_deg` field contains flange XYZ and model RPY. RealArm
continues to return SDK coordinates with its existing settings; its orientation
is not assumed interchangeable with model RPY.

## Historical comparison, October 28, 2026

Inputs are transcribed observations in `pi_verification.md` and `standby_plan.md`,
not fresh physical measurements. The reference query explicitly reported base
frame, flange end type and zero tool/world references. These settings were not
captured for every other snapshot; their frame equivalence is an assumption.

| Observation | J1-J6 (degrees) | Reported XYZ (mm) | Predicted XYZ (mm) | Error (mm) |
| --- | --- | --- | --- | --- |
| Standby planning | 108.01, -19.59, 11.07, 145.81, 76.28, -16.25 | 20.5, -1.3, 290.5 | 20.337, -0.061, 291.074 | 1.375 |
| Reference query | 144.84, -14.76, 11.16, 122.95, 86.74, -45.79 | 36.8, -4.1, 307.4 | 35.685, -3.003, 307.519 | 1.569 |
| Initial telemetry | -2.63, 8.7, -26.36, 130.34, 99.14, 9.31 | -54.8, -15.8, 304.4 | -53.103, -16.180, 304.265 | 1.745 |
| Packaged status | 43.06, -14.58, 11.07, 153.8, 79.8, -44.91 | 4.9, -20.7, 280.9 | 5.771, -20.257, 281.667 | 1.242 |

These small model-to-controller discrepancies support continued offline work.
They do not measure physical accuracy: the controller's coordinate report may
itself be calculated from joint encoders and nominal geometry.

## Next validation milestone

The latest [workspace validation baseline](workspace_validation.md) incorporates
the user's physical ready-point measurement, clarified mat origin/directions,
and unresolved flange-to-gripper reference discrepancy. Read it before using
the earlier candidate tool offset below.

### Fresh capture results and provisional gripper geometry, October 28, 2026

The user supplied `fk_ready_20261028T231337_139681Z.json` and
`fk_standby_20261028T231423_085621Z.json`. Recalculation reproduced all saved
errors. Each contains three identical paired joint/coordinate readings, zero
reported joint spread, and base/flange settings with zero tool/world references.
Standby error is 1.070 mm (predicted-minus-reported Z +1.039 mm); ready error is
2.674 mm (Z -2.664 mm). Maximum target-angle offsets are 0.96 and 0.88 degrees,
respectively. No fixed base-height correction was fitted to these different
pose-dependent discrepancies. Fresh controller comparison is complete at these
two poses; physical position validation remains pending.

The user subsequently measured pickup-point extension from the flange face as
3 15/16 inches (100.0125 mm), offset 1/4 inch above center (6.35 mm), and the
robot mounting surface 5/16 inch above the table (7.9375 mm). Five supplied
photos were explicitly confirmed taken at saved ready. The photographs establish
mounting context, not calibrated dimensions or coordinate axes.

`config/tool_geometry.provisional.json` records these measurements and assumptions.
Assuming outward is flange +Z, and above is base +Z at nominal ready, the candidate
flange-local pickup point is `[4.490128, 4.490128, 100.0125]` mm. The 6.35 mm
upward displacement splits across flange X/Y because of the nominal -135-degree
J6 orientation. This is a position offset only, not a calibrated tool orientation.
The flange-face/model-origin correspondence and zero additional sideways offset
are assumptions. This configuration is not applied automatically.

Use `pose.point_in_base(offset_mm)` or the explicit offline CLI option:

```sh
python -m bucke_arm fk --angles 89.12 -0.52 0.35 0 90.7 -134.29 --tool-offset-mm 4.490128 4.490128 100.0125
```

`pickup_point_base_mm` is separate from flange XYZ; `--reported-xyz` continues
to compare flange coordinates. No SDK settings are written. FakeArm continues
to report flange pose until a physical tool offset is validated.

For the measured ready angles, candidate pickup-point base XYZ is
`[-81.975, 1.082, 424.489]` mm. **If** the model base origin is at the physical
mounting surface, predicted table-to-pickup-point height is 432.427 mm (17.025
inches). Using reported flange Z with the model-rotated offset instead gives
435.090 mm (17.130 inches). These are conditional estimates, not validated
table coordinates. Next measure vertically from tabletop to the center of the
intended gripping contact area at stationary ready, without touching the arm.
This checks the combined vertical geometry; a match alone would not establish
the complete tool transform or base-to-mat alignment.

### Capture fresh standby and ready samples on the Pi

Transfer the updated arm folder from Windows PowerShell, replacing `PI_ADDRESS`:

```powershell
scp -r "C:\Users\heath\OneDrive\Documents\Repos\buck-e-capstone\arm" er@PI_ADDRESS:/home/er/buck-e-capstone/
```

Close the manual controller and any other program using the serial connection.
With the arm already stationary at standby, run on the Pi:

```sh
cd ~/buck-e-capstone/arm
python3 -m bucke_arm.fk_capture --pose standby
```

Use the existing manual controller's R command for the confirmed standby-to-ready
transition, then exit the controller. At stationary ready, run:

```sh
python3 -m bucke_arm.fk_capture --pose ready
```

If already at ready, capture ready first; use the existing S transition and exit
the controller before capturing standby. The capture tool sends queries only;
it never moves the arm or changes frame, tool, calibration or torque settings.
It uses RealArm's verified SDK/version/connection checks and does not substitute
simulated readings if hardware access fails.

Each run creates a uniquely named JSON file in `fk_reports/` and prints its full
path plus minimum, maximum and mean XYZ error. `--output PATH.json` selects a
new file; existing files are never overwritten. Keep both reports for review.

The capture checks motion/error state before and after each sample, brackets
the coordinate query with joint-angle reads, and reads reference settings at
both ends. Base frame, flange end type and zero tool/world references are
required, consistent with the previously observed configuration. Unsupported
settings are reported without changing them. All angle readings must match
the selected saved pose within the existing 3-degree tolerance and have no
more than 0.25-degree total per-joint spread across the run. This spread is
a sampling-consistency check, not a physical accuracy threshold.

There are three samples, spaced by 0.5 seconds after each completed sample.
Readings remain sequential rather than atomic. Full six-component SDK
coordinates and reference settings are saved, but only XYZ is compared. A
failure ends capture, returns exit code 1 and saves completed samples plus a
failure message; it does not issue stop, retry or recovery commands. A complete
capture means usable samples were collected, not that physical accuracy passed.

Implementation query meanings were checked against the manufacturer's
[pymycobot 4.0.6 source](https://github.com/elephantrobotics/pymycobot/blob/v4.0.6/pymycobot/mycobot280.py).
Hardware execution at standby and ready is recorded above.

At already authorized poses, collect stationary paired joint/coordinate samples
with reference frame, end type and tool/world reference settings recorded.
Compare XYZ using the command above. Independently measure physical flange or
tool positions in a known base frame and establish an accuracy requirement
before choosing an acceptance threshold. Then verify orientation conventions,
the gripper-tip transform, mat/base alignment and table height.

FK validates input shape and finite numbers, but does not enforce motion limits
or model gripper geometry, collisions, trajectories, deflection, motor loads
or thermal behavior. It is not workspace validation or authorization for new
movement.
