# Workspace validation baseline, October 28, 2026

The user confirmed mat (0,0) is below J1's rotation axis. In the latest overhead
photo, the Sharpie tip marks mat +X and the pink pencil tip marks mat +Y.
Grid units are inches; mat Z is defined upward from the tabletop. The physical
ready pickup point is reported at (4,0) inches with height 17 3/16 inches:
`[101.6, 0, 436.5625]` mm. The gripper's visible start is reported above (0,0).
These are approximate user measurements, with no stated measurement uncertainty.

## Model audit

The pinned manufacturer URDF used by FK ends at `joint6_flange`. It does not
include this gripper or its mounting relationship. Its visual mesh placements
do not establish that the user's visible gripper start is the mathematical
flange origin. The audit did not establish a correction or an exact source of
the observed discrepancy. The original 100.0125 mm extension remains recorded,
but its identification as a model-flange-to-pickup distance is unverified.

Approximate base-to-mat alignment is yaw 180 degrees. If model base origin is
at the physical mounting surface, mat coordinates would be:

```text
mat_x = -base_x
mat_y = -base_y
mat_z = base_z + 7.9375 mm
```

This is a hypothesis for offline comparison, not a calibrated transform. It
comes from one physical ready observation approximately along a single axis;
no additional off-axis point verifies yaw.

## Ready comparison

Using measured ready angles `[89.12,-0.52,0.35,0,90.7,-134.29]` and the original
candidate flange-local tool point `[4.490128,4.490128,100.0125]` mm:

| Mat coordinate | Predicted (mm) | Observed (mm) | Predicted minus observed (mm) |
| --- | --- | --- | --- |
| X | 81.975 | 101.600 | -19.625 |
| Y | -1.082 | 0.000 | -1.082 |
| Z | 432.427 | 436.563 | -4.136 |

Total discrepancy is 20.086 mm. This candidate does not reproduce the physical
ready observation. A single point cannot isolate tool offset, base height,
axis alignment and measurement/reference errors. No fitted tool offset, global
base correction or SDK reference setting has been applied.

`config/workspace.provisional.json` records the baseline and calculated residuals.
`config/tool_geometry.provisional.json` retains the original dimensions alongside
the clarified reference ambiguity. Neither file is loaded into movement code.

## Next useful observation

### Fresh standby observation received later October 28

The user corrected the physical standby pickup point to mat (-3.5,+4) inches,
height 7 1/8 inches: `[-88.9,101.6,180.975]` mm. Earlier (-4,+1) and (-4,+4)
estimates are superseded. The fresh capture
`fk_standby_20261029T000002_737108Z.json` has three identical samples at
`[-0.61,0,89.91,89.38,89.56,-134.29]` degrees, unchanged base/flange reference
settings and zero references. Recomputed flange XYZ error is 1.080 mm. The UTC
filename is October 29; the observation is October 28 in America/New_York.

Under the original tool offset and hypothesized 180-degree yaw, predicted mat
pickup XYZ is `[97.303,-83.029,177.540]` mm. Horizontal signs disagree with the
physical observation; near-equality of height does not validate the transform.
The current joint readings rule out a large unintended J1 difference relative
to the earlier nominal standby report as the simple explanation. They do not
independently establish physical joint zero calibration.

The earlier 180-degree alignment was inferred from one ready observation and
must remain a hypothesis. Next inspect an overhead view of this stationary
standby pose with the established mat-axis markers visible, to compare physical
link/gripper directions against the model before selecting additional physical
measurements or motion. No SDK settings or offsets have been changed.

The user then supplied that overhead standby view. In this view the Sharpie tip
points toward the table front (image bottom, mat +X), and the pink pencil tip
points image right (mat +Y). The pickup point visibly lies behind J1 and to its
right, consistent with the user's negative-X/positive-Y measurement. Therefore
reversing the supplied measurement signs is not a supported resolution. The
photo is not a calibrated image and was not used to estimate new dimensions.

An offline exploratory calculation using negated model J1, identity horizontal
base/mat alignment and roughly 120 mm tool extension can reproduce the gross
directions in both reported poses. This is an unvalidated hypothesis, not a
joint convention correction. The manufacturer's pinned `slider_control.py`
passes ROS joint angles directly to `send_angles`, and `follow_display.py`
publishes SDK radians unchanged. The latter separately rotates SDK coordinate
markers as ROS x=-SDK y, ROS y=SDK x, which illustrates a frame-convention issue
that must be reconciled with the exact description/launch model before drawing
conclusions. These scripts were inspected as source text only; they were not
executed. No joint sign, command angle, FK convention or hardware setting was
changed on the basis of the exploratory fit.

The physical observations are now the retained references. The next software
investigation is a comparison of physical link directions with the exact robot
description and SDK frame definitions; further repetition of these same mat
measurements would not resolve those conventions.

The user explicitly confirmed that the base remained fixed throughout both
sets of photos and measurements. The pinned 280 Pi `slider_control.launch` and
`mycobot_follow.launch` both select the same URDF used here. Thus a different
default ROS description or a moved base is not supported as the explanation.

[Top-view comparison](evidence/2026-10-28/frame_comparison.svg) shows the joint
pivot chain, original candidate pickup offset and physical measured points for
both poses under the same original 180-degree alignment. The diagram is a
diagnostic, not body/collision geometry. It demonstrates the failed hypothesis;
it does not calibrate a replacement. No single yaw correction can fix the
opposite standby direction while retaining the current ready alignment.

The reconciliation procedure is to compare the shoulder/elbow/wrist chain
separately from the attached gripper, establish one fixed base-frame alignment,
then establish one flange-local pickup offset that explains both orientations.
Joint signs, frame rotations and tool offsets must not be independently changed
for each pose. If the chain itself fails to match the photos under one fixed
alignment, investigate physical joint-zero/direction correspondence before
fitting a tool offset. Photographs without visible joint-axis landmarks do not
fully determine this correspondence. A qualified conclusion may require an
independent flange/axis reference observation; repeating pickup measurements
alone is not a substitute. Full calibration is not claimed complete.

At another already verified stationary pose with unchanged gripper mounting,
record the same physical pickup-point XYZ in mat coordinates and capture the
paired joint/flange readings. Saved standby is a candidate only if the existing
transition is verified for the currently attached gripper. No new movement
route is selected by this document.

At stationary standby, the existing read-only command is:

```sh
cd ~/buck-e-capstone/arm
python3 -m bucke_arm.fk_capture --pose standby
```

Measure the vertical projection of the same gripping contact point onto the
mat for X/Y, and its vertical height above the tabletop for Z. Keep signs and
units explicit. This adds a second orientation to help distinguish a rotating
tool-offset discrepancy from a fixed base-frame discrepancy. It is still
insufficient by itself for complete calibration of all unknown transforms.

The prior intended pickup mat (7,2) inches gives horizontal coordinates
`[177.8,50.8]` mm. Its object grasp height, tool orientation, reachability,
full-arm clearance and approach route remain to be established before motion.

## Actual flange reference correction, November 3, 2026

The user identified the old measuring reference as the camera flange, not the
actual robot flange. The corrected actual-flange-to-gripper-midpoint distance
is approximately 4 11/16 inches (119.0625 mm), superseding their initial
4 1/2-inch estimate. Historical candidate geometry and comparisons above are
retained; 100.0125 mm is not a verified actual-flange extension.

For a conditional offline comparison, replacing only candidate local Z with
119.0625 mm while retaining the old transverse offset and 180-degree mat yaw:

| Pose | Predicted mat XYZ (mm) | Predicted minus observed (mm) |
| --- | --- | --- |
| Ready | 101.025, -1.141, 432.427 | -0.575, -1.141, -4.135 |
| Standby | 97.246, -102.078, 177.542 | 186.146, -203.678, -3.433 |

Ready horizontal agreement improves substantially, but standby still has
opposite horizontal signs. This does not establish a corrected complete tool
transform: the new distance is assumed along model flange +Z and midpoint is
assumed identical to the previously measured pickup point. The earlier 6.35 mm
transverse offset may also reference the camera flange and needs clarification.
Next establish those reference relationships before revising the tool vector.
No active FK conventions, motion code or SDK settings were changed.

## Standby actual flange observation

The user located the actual flange center at approximately mat (-4, -1)
inches and reported its face points toward mat +Y. The gripper midpoint was
approximately (-4, +4) inches in this observation. These are rough landmarks.

At the captured standby angles, FK predicts flange XY (-97.4, -17.0) mm
in model base coordinates and flange +Z projects almost exactly along base
+Y. Direct horizontal alignment gives flange XY (-3.84, -0.67) inches
and the observed facing direction. The provisional 180-degree alignment
instead gives (+3.84, +0.67) inches and flips the facing direction to -Y.
This supports direct horizontal alignment at standby. Under that alignment,
the earlier corrected ready prediction would land near mat X -101 mm versus
observed +102 mm. Check the actual flange center and facing direction at ready
before adopting a global frame change.

## Ready actual flange observation

The user located the actual flange center near mat (-1, 0) inches at ready
and reported that its face points toward mat +X. With direct horizontal
alignment, FK at the captured ready angles gives flange XY (+0.71, +0.03)
inches and a flange +Z direction near mat -X. As an offline diagnostic,
negating only the FK input for J1 changes this to (-0.71, -0.05) inches
and a facing direction near mat +X. Standby J1 is near zero, so the
same diagnostic has little effect there. This makes J1 direction a
candidate to verify against an independent physical J1 reference.

The user's Pi controller screenshot subsequently showed ready target J1 -90
degrees, final ready angles [-89.12, 0.61, 0.61, 0.61, 90.79, -134.2],
and final standby J1 -0.96 degrees. Using the ready readings as displayed
gives flange XY (-0.71, +0.23) inches with the face pointing nearly +X
under direct alignment, consistent with the physical ready observation.
The negative reported angle accounts for the directional mismatch without
an FK sign conversion. The earlier positive-J1 capture represents different
input and should not be paired with these physical ready landmarks. The Pi
menu's negative ready target supersedes the former positive target. The
primary checkout now uses the user-confirmed -90-degree ready target.
