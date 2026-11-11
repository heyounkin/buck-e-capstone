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

## Joint Control gripper readback, November 10, 2026

After a J1 command from a displayed -89.20 to a target of -90.00 degrees,
Joint Control reported mat XYZ (3.98, 0.03, 16.77) inches. The operator
corrected the physical gripper measurement to X 4 inches and reported Z
approximately 17.5 inches. Physical Y and the final six joint angles were not
recorded in the screenshot. The estimate differs by 0.02 inch in X at the
reported precision and is about 0.73 inch low in Z.

The subsequent Ready capture `fk_ready_20261110T214424_366112Z.json` has three
identical stationary samples at [-89.29, 0.35, -0.70, -0.70, 89.20, -134.29]
degrees. The controller reports base-frame flange XYZ [-17.7, -2.1, 420.8] mm
with zero tool and world references. Model flange XYZ is
[-18.00, -2.11, 418.11] mm, 2.70 mm away in total and 2.69 mm lower in Z.
With the provisional mat alignment and height offset, the model flange is
at (-0.71, -0.08, 16.77) inches and the model jaw midpoint is at
(3.98, -0.09, 16.77) inches. The controller flange Z converts to 16.88 inches
under the same mat assumptions. Substituting the controller flange position
would raise the jaw estimate by only about 0.11 inch.

The operator then measured the actual flange center at 17 3/8 inches above
the tabletop in Ready. Against the captured angles, the model flange is
approximately 0.60 inch low and the controller flange, converted with the
same provisional mat offset, is approximately 0.50 inch low. The reported
17 1/2-inch jaw height puts the jaw 1/8 inch above the actual flange center;
the current axial-only tool model puts them at almost the same height.
The total approximately 0.73-inch jaw discrepancy therefore splits into
approximately 0.60 inch at the flange and 0.13 inch in flange-to-jaw height.
This single-pose comparison does not isolate base height, joint-zero,
physical reference or model-geometry errors. A fresh stationary standby
flange/jaw observation with paired angles would test whether the flange
height discrepancy persists across orientations.

### Fresh standby capture and reported J2 give

The subsequent standby capture `fk_standby_20261110T222044_224020Z.json`
completed three stationary samples, all at
[-0.70, 0.79, 90.00, 89.38, 89.29, -134.29] degrees, with unchanged zero
references and base-frame flange coordinates. The model/controller flange
position difference is 1.071 mm. The operator reported mat XYZ
(-3 7/8, 4, 6 7/8) inches, provisionally interpreted as the jaw midpoint;
a separate current standby flange height was not supplied.

The current jaw estimate is (-3.862, 4.025, 7.180) inches, giving
predicted-minus-observed errors of (+0.013, +0.025, +0.305) inches.
Thus Ready jaw Z is approximately 0.73 inch low, while this standby jaw Z
is approximately 0.31 inch high. A single global Z translation cannot
reconcile both reported jaw heights under the current tool assumptions.

The operator also noticed give at J2. All captured J2 readings remain
0.79 degrees; the capture does not establish the amount, cause or direction
of the physical give, or whether it changes under load. Physical movement
with unchanged encoder readings would limit a model based only on those
angles. This observation requires clarification before fitting corrections.

The operator clarified that, at Ready with the servo active, physical J2
movement changes its reported angle from approximately 0.26 to 0.70 degrees,
a range of 0.44 degree. Holding the other angles from the Ready capture
fixed, FK predicts jaw displacement across that range of approximately
(-0.0010, +0.0845, +0.0009) inches in provisional mat XYZ. Predicted flange
height changes by approximately 0.0004 inch. Thus the observed reported-angle
range contributes primarily horizontal variation at this pose and does not
account for the approximately 0.73-inch Ready jaw-height discrepancy.
The observation does not identify the cause of the physical give or exclude
additional physical deflection that the angle readings do not resolve.

The operator then supplied the actual standby flange-center height of
7 3/8 inches. Combined with the reported standby jaw height of 6 7/8 inches,
the physical jaw is 1/2 inch below the flange center at this orientation.
The captured-angle model flange height is 7.180 inches, approximately
0.195 inch below the measured flange. The model jaw is approximately
0.305 inch above the measured jaw. By comparison, Ready has approximately
0.601 inch of flange-height error and a physical jaw 1/8 inch above the flange.
These observations constrain both base/flange correspondence and the
orientation-dependent tool offset; they do not establish a single global
height correction.

The operator supplied approximate flange-center mat XY at Ready (-1, 0)
inches and at Standby (-4, -9/16) inches, explicitly correcting Standby X
from +4 to -4. Together with the reported heights, the measured flange
positions are (-1, 0, 17.375) and (-4, -0.5625, 7.375) inches. The corresponding
captured-angle model positions under the provisional mat transform are
(-0.709, -0.083, 16.774) and (-3.861, -0.662, 7.180) inches.

Their measured separation is 10.455 inches versus the model's 10.115 inches,
a difference of 0.341 inch (8.66 mm). Rigid frame transformations preserve
this distance, so no rigid base-to-mat transform can match both nominal
measured flange points exactly. Measurement uncertainty was not quantified.
The reported Ready flange-to-jaw displacement implies a distance of at least
5.002 inches (jaw Y unspecified), whereas the reported Standby displacement
has length 4.592 inches. These approximate landmarks also require reconciliation
before fitting a single rigid tool vector. The current measurements therefore
provide a consistency check, rather than a completed calibration.

The operator confirmed both flange measurements refer to the robot output
flange, and later revised Ready flange XY to (-1/2, -5/8) inches. The later
Standby message listed two Y entries; the first is provisionally interpreted
as X -4 inches and the second as Y -9/16 inch, consistent with the preceding
explicit Standby X correction. Using the previously reported flange heights
of 17 3/8 and 7 3/8 inches, the revised measured flange separation is
10.595 inches versus the model's 10.115 inches. This increases the distance
discrepancy to approximately 0.481 inch (12.20 mm). The heights and interpretation
of the duplicated axis label require confirmation before further calibration.

The operator subsequently confirmed Standby flange Z of 7 3/8 inches and
corrected Ready flange Z to 17 1/8 inches. The current flange observations
are therefore Ready (-1/2, -5/8, 17 1/8) and Standby
(-4, -9/16, 7 3/8) inches. Their separation is 10.359 inches, giving a remaining
0.245-inch (6.22 mm) difference from the model's 10.115 inches. Under the
original provisional mat transform, the model flange is approximately
0.351 inch low at Ready and 0.195 inch low at Standby. With Ready jaw height
still reported as approximately 17 1/2 inches, its physical height above
the flange is now 3/8 inch. Physical Ready jaw Y remains unreported.

### Provisional rigid fit from the completed measurements

The operator supplied Ready jaw Y of -5/8 inch, using the midpoint between
the closed gripping pads. The latest measured mat coordinates in inches are:

| Pose | Robot output flange center | Jaw midpoint |
| --- | --- | --- |
| Ready | (-0.5, -0.625, 17.125) | (4, -0.625, 17.5) |
| Standby | (-4, -0.5625, 7.375) | (-3.875, 4, 6.875) |

Both flange references are the robot output flange, upstream of the camera
mount. These values supersede the earlier approximate measurements above.
The observed flange-to-jaw distances are 4.516 inches at Ready and 4.592 inches
at Standby, a difference of 1.93 mm. A rigid tool has one fixed length;
these measurements retain uncertainty.

An offline least-squares fit used the captured joint angles, the unchanged
pinned FK geometry, one rigid base-to-mat transform and one fixed flange-local
jaw vector. All twelve measured coordinate components had equal weight.
Three different initial yaw values converged to the same residual sum.
The exploratory parameters are:

- Base-to-mat RPY: [3.22915, 1.84952, -0.81167] degrees using Rz Ry Rx.
- Base-to-mat translation: [-8.60493, 10.73315, 12.66726] mm.
- Flange-local jaw midpoint: [15.16635, 12.08422, 114.89588] mm.

| Pose/reference | Fitted mat XYZ (inches) | Position residual (mm) |
| --- | --- | --- |
| Ready flange | (-0.531, -0.585, 16.943) | 4.790 |
| Ready jaw | (4.015, -0.637, 17.559) | 1.563 |
| Standby flange | (-3.992, -0.574, 7.439) | 1.674 |
| Standby jaw | (-3.866, 3.984, 6.934) | 1.563 |

The fit requires approximately 3.72 degrees between modeled base +Z and mat
+Z. That tilt has not been physically verified and may absorb measurement,
joint alignment or compliance errors. The jaw vector represents the combined
camera mount and gripper geometry. These values are an offline candidate;
Joint Control used its original provisional offset and alignment at this stage.
Independent observations are required to establish accuracy beyond the two
fitted poses. A low residual on fitted observations is not independent
workspace validation.

The operator confirmed the robot mounting surface is parallel to the tabletop.
This rules out interpreting the unconstrained fit's 3.72-degree base tilt as
the physical mounting alignment. Repeating the equally weighted fit with
roll and pitch fixed to zero gives candidate yaw -2.73492 degrees,
translation [0.08989, -7.07947, 14.10556] mm, and tool vector
[6.96686, 11.65140, 120.24856] mm. Position residuals are 9.618 mm at Ready
flange, 5.853 mm at Standby flange and 2.246 mm at each jaw observation.
The remaining flange residuals motivate checking the robot-specific URDF
origins, axes, fixed transforms and output-flange reference against the pinned
geometry currently hardcoded in FK. An updated robot-specific model has not
yet been supplied.

### Supplied camera-flange and gripper URDF comparison

The operator supplied
`mycobot_280_pi_camera_flange_plus_gripper_unchanged_transforms.urdf` from
Downloads (SHA-256
`de72e4da5817bbb3aadd2ca78174a6332122068fd6698110b6916d735d9745f8`).
All six arm revolute-joint origins, RPY rotations and +Z axes match the
hardcoded pinned FK geometry. The g_base-to-joint1 fixed transform is identity
in both. Directly composing the supplied arm chain at both captured poses
reproduces the current FK matrices to numerical precision. This file therefore
supplies no changed arm geometry to resolve the flange-position residuals.

It adds two relevant fixed attachment transforms (xyz converted to mm):

| Parent to child | XYZ (mm) | RPY (radians) |
| --- | --- | --- |
| joint6_flange to camera_flange | (0, 0, 10) | (1.579, 0, 1.5708) |
| camera_flange to gripper_base | (0, 40, 0) | (0.0082, -1.5708, 0) |

Their combined flange-to-gripper-base translation is approximately
[0.328143, 0.000001, 49.998654] mm. The full compound rotation must accompany
that translation. The file also describes the gripper mechanism and mimic
joints, but defines no named pad-contact point or midpoint tool frame.
Joint origins and visual/collision mesh placements do not by themselves
identify the physical pad contact centers or the gripper closure calibration.
A pad midpoint requires explicit local contact geometry and gripper state,
or a verified rigid midpoint approximation for the closed-pad condition.
The supplied URDF references external DAE meshes that were not included in
the attachment. The correspondence between its nominal output-flange frame
and the measured physical face center remains a physical/mesh verification
question. The URDF comparison changes no runtime transforms or movement
limits.

### ROS 2 Humble mesh inspection

The operator identified the manufacturer ROS 2 repository. Meshes and URDFs
were downloaded under Downloads/bucke-model-review/d42ff61a7812, pinned to
mycobot_ros2 commit d42ff61a78122c79246623391540d75738b03b23.
The joint7 mesh is byte-identical to the earlier pinned ROS 1 version.
The left gripping-finger mesh has identical vertex coordinates and scene
matrices across the two revisions, despite different file bytes.

Mesh inspection applied COLLADA units and scene/instance transforms before
the URDF visual origins. The joint7 outer planar face is at mesh Z 13.7 mm;
the URDF visual origin shifts this by -12 mm, placing that modeled face at
flange-local Z +1.7 mm. This provides a model face reference to compare with
the physical output face. At the measured Ready and Standby orientations,
this offset acts almost horizontally and does not explain the large jaw Z
residuals.

Candidate inward pad faces were identified as the large planar faces of the
pad mesh components, approximately 509.86 square mm each. Their area centroids
in the left1 and right1 link frames are respectively
[18.707451, 39.986547, -8.2] and [-19.922323, 40.358562, -8.2] mm.
Using the supplied URDF's controller lower limit of -0.74 rad, including
mimic joints, yields a pad-centroid midpoint in gripper_base of
[-0.607436, 75.193825, -8.2] mm. Through the supplied camera/gripper chain,
that is flange-local [0.338117, 7.583145, 125.259641] mm. This assumes the
identified planar faces are the measured contact surfaces. Their centroid
X separation at that joint limit remains 3.67 mm, so the lower limit alone
does not establish the hardware's fully closed configuration or pad contact.
The controller-angle mapping to the physical gripper remains unverified.

The repository camera-flange model uses RPY [1.579, 0, 0] radians for the
robot-to-camera fixed joint, while the supplied combined file uses
[1.579, 0, 1.5708]. The supplied camera is therefore rotated an additional
90 degrees about the parent frame Z axis. The stock adaptive-gripper file
mounts the gripper directly on the robot flange with XYZ [0, 0, 34] mm and
RPY [1.579, 0, 0], representing a different attachment configuration from
the supplied combined camera/gripper chain. The physical mounting orientation
must determine which attachment transforms apply.

### Current pad-coordinate reporting correction

The operator clarified the camera is perpendicular to the gripper and stands
upright at Ready. That description does not alone identify the camera-link
RPY axes. The current reporting correction instead uses the combined
flange-local closed-pad vector from the fit constrained to a parallel base,
so it includes the mounted camera/gripper assembly through the observed
pad positions. The arm joint origins and axes remain the pinned URDF values.

`kinematics.gripper_midpoint_mat` uses the level-base fit parameters recorded
above: mat yaw -2.734920596 degrees, translation
[0.089888631, -7.079468096, 14.105560122] mm and closed-pad offset
[6.966863290, 11.651398531, 120.248563380] mm. Joint Control passes the final
actual angle readings to this function. It applies one rotating tool vector
and one fixed mat transform for every pose. The menu labels the fit as
provisional and uses the closed-pad reference even when the jaws are open.
The observed Ready/Standby pad residuals are approximately 2.246 mm each.
The flange residuals of 9.618 and 5.853 mm remain unresolved; the fitted tool
vector may absorb some error from nominal flange geometry or measurements.
This corrects the displayed estimate at the measured poses and supplies a
consistent transformation elsewhere, with accuracy outside those fitted
poses still requiring independent validation. The correction does not provide
a collision model or establish motion targets as verified.

### First post-transfer standby comparison

The operator's Pi screenshot after the reporting correction shows Ready
pad estimate (4.00, -0.87, 17.53) inches following an individual joint
command, then Standby confirmed with estimate (-3.68, 3.99, 6.96) inches.
The operator measured that current Standby jaw midpoint at
(-4, 3 15/16, 7) inches. Predicted-minus-measured error from the rounded
display is (+0.320, +0.0525, -0.040) inches, approximately
(+8.13, +1.33, -1.02) mm. This is a separate post-transfer observation,
with approximately 8.3 mm total pad-position error, larger than the residual
on the fitted Standby sample. The actual six joint angles for this run
were not included in the screenshot. Pairing those angles with the current
measurement is required before attributing the change to code, geometry,
reference measurements or physical repeatability.

The subsequent read-only status screenshot reports actual angles
[-0.87, 0, 90, 89.38, 89.91, -134.29] degrees and SDK flange coordinates
[-97.1, -16.3, 174.7] mm. Feeding those angles to the local corrected reporter
gives (-3.699, 3.987, 6.961) inches, slightly different from the earlier menu
sample's (-3.68, 3.99, 6.96) display. The current measured pad position gives
approximately (+7.65, +1.26, -0.99) mm error, 7.81 mm total. The flange FK
and SDK positions differ by 1.11 mm, which compares two calculated positions
rather than establishing physical accuracy. The source
measurement/model comparison therefore persists with the actual joint readings;
it is not explained by using the commanded targets in FK. Relative to the
fit's earlier Standby capture, J2 changed from 0.79 to 0 degrees and J5 from
89.29 to 89.91 degrees, so the test pose is a different actual configuration.
This post-fit observation limits confidence in the two-pose calibration.

The operator confirmed the current physical robot flange center remains
(-4, -9/16, 7 3/8) inches. With the latest pad midpoint (-4, 3 15/16, 7)
inches, its flange-to-pad vector is (0, 4.5, -0.375) inches. Its length is
4.516 inches, matching the Ready observation's (4.5, 0, 0.375) inch vector.

The operator also demonstrated J5 readings changing from 91.14 to 88.76
degrees while investigating physical give. The screenshot shows no movement
command sent between those readings. At the other angles from the current
Standby status, varying model J5 across that 2.38-degree range moves the
predicted pad approximately (+0.2710, -0.0112, -0.0029) inches. This is
approximately 6.9 mm along X, comparable to the observed position discrepancy.
It establishes sensitivity, rather than proving the cause of a measurement
made at a different instant. The actual-angle model accounts for the sampled
J5 value, but a previously printed XYZ can become stale as the joint changes.

Joint Control now pairs every printed XYZ with the exact reported-angle
snapshot used and offers P to refresh the angles and estimate without
commanding motion. Measurements for further calibration should be paired
with these current snapshots; the joint's mechanical or holding behavior
has not been diagnosed or altered.

The operator subsequently corrected the physical Standby jaw X measurement
from -4 inches to -3 12.5/16 inches, meaning -3.78125 inches. Keeping the latest
reported Y 3 15/16 and Z 7 inches, the corrected observed pad point is
(-3.78125, 3.9375, 7) inches. Against the most recent paired status angles
[-0.87, 0, 90, 89.38, 89.91, -134.29], the model predicts
(-3.69891, 3.98705, 6.96098) inches. Predicted-minus-observed errors are
approximately (+2.09, +1.26, -0.99) mm, 2.63 mm total. This supersedes the
earlier approximately 7.81 mm error using X -4 inches. It compares with the
last recorded status snapshot; a newer P snapshot has not been supplied.
The correction improves agreement through a corrected physical measurement,
rather than a change to calibration parameters or the read-only P feature.

A global Z correction of +0.73 inch would move the modeled October 28 standby
height from 7.24 to 7.97 inches, farther from the measured standby height of
7.125 inches. This comparison uses the current Joint Control assumptions
(aligned horizontal axes, 5/16 inch base height, and 4 11/16 inches along
flange +Z).

To separate a flange-to-jaw offset error from a base-to-mat error, pair all
six stationary joint readings with mat XYZ measurements of both the actual
flange center and jaw midpoint at the same pose. The difference between those
two measured points is independent of any fixed base-to-mat translation.
