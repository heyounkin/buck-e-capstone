# J2 commanded-versus-reported position discrepancy

## Observed configuration

- myCobot 280 Pi; pymycobot 4.0.6; Atom firmware 7.3.
- Serial /dev/ttyAMA0, 1000000 baud.
- Current tool: mounted camera/gripper assembly; measured assembly mass not supplied.
- Single-joint tests at SDK speed 10; gripper raised clear of table before these tests.
- Pickup remains disabled in the primary Windows checkout after table contact and
  staggered joint arrival. Pi lockout deployment has not been confirmed.

## Command response

| J2 requested (degrees) | J2 reported (degrees) | Reported minus requested | Observation |
| --- | --- | --- | --- |
| +2.00 | +0.61 | -1.39 | Moved upward from -19.68 |
| -19.00 | -19.59 | -0.59 | Approached from +0.61; repeated P reads stable |
| -18.40 | -19.59 | -1.19 | No visible or reported movement |
| -17.50 | -19.16 | -1.66 | Moved +0.43; subsequent P stable |

The application uses a 3-degree pose-completion tolerance, which can accept
these residuals. This explains the menu's return, not the physical shortfall.
No fixed correction or specific deadband threshold has been established.

## Latest read-only controller capture

- Actual J1-J6: [36.65, -19.16, -117.59, 44.47, 0.08, 160.75].
- Angle-plan reply: [37.28, -17.5, -117.11, 45.68, 0.08, 161.63].
- Refresh mode: 1.
- J2 enabled: 1.
- J2 raw encoder: 2266.

The J2 angle-plan entry agrees with the latest command while actual J2 is
1.66 degrees lower. This supports the command reaching the controller's plan,
not successful physical tracking. The SDK's angle-plan API is only briefly
documented; it is not an independent measurement of output-link orientation.
The raw encoder has not been assigned an independently verified output-angle
conversion for this particular assembly.

## Raw servo parameter capture

Addresses: [7, 21, 22, 23, 24, 26, 27].

| Joint | Raw values |
| --- | --- |
| J1 | [0, 15, 20, 0, 0, 3, 3] |
| J2 | [0, 32, 8, 0, 0, 3, 3] |
| J3 | [0, 32, 8, 0, 0, 3, 3] |
| J4-J6 | [0, 25, 25, 0, 0, 3, 3] |

J2 and J3 match; the values also match the manufacturer's demo parameter array.
Neither fact establishes appropriate factory settings or mechanical health.
No parameters, zero calibration, holding mode or firmware were changed.

## Physical observations and limits

With power and servos engaged, the operator reported a bounded hand-movable
J2 range of approximately -21.26 to -20.74, returning to -21.26 after release,
and a much larger physical tip-height change than nominal FK predicts. The
fingers were touching the table during that earlier observation. This is not
an unloaded holding or backlash calibration. There is no reported uncontrolled
sag at rest. The cause remains unresolved between control behavior, load,
mechanical compliance/play and measurement correspondence.

## Questions for model-specific service guidance

1. Is a stable 0.59-1.66-degree reported J2 following error expected under this
   load and firmware, including no response to the -18.40-degree request?
2. What are the correct register definitions, units and factory values for this
   unit's J2, especially 21-24 and 26-27? Do not infer them from another model.
3. What read-only load/current diagnostics and supported mechanical inspection
   distinguish control deadband/stiffness from coupling or joint play?
4. If tuning is warranted, what reversible, model-specific procedure and limits
   apply? Original raw values above must be preserved before any approved write.

No further target increases, automatic retries or arbitrary PID/torque changes
are recommended from these observations alone. Resolve J2 tracking before
recalibrating pickup or enabling a new near-table trajectory.

## Manufacturer references

- [pymycobot v4.0.6 parameter example](https://github.com/elephantrobotics/pymycobot/blob/v4.0.6/demo/pid_read_write.py)
- [pymycobot v4.0.6 MyCobot280 implementation](https://github.com/elephantrobotics/pymycobot/blob/v4.0.6/pymycobot/mycobot280.py)
- [myCobot 280 Pi hardware troubleshooting](https://docs.elephantrobotics.com/docs/mycobot_280_pi_en/4-SupportAndService/9.Troubleshooting/9.3-hardware.html)


## Accessory identification follow-up

The operator states the camera and gripper are the manufacturer's website
accessories. The repository model depicts an adaptive gripper and camera
flange, suggesting the standard myCobot adaptive gripper and Camera Flange 2.0;
the operator subsequently confirmed these are the correct models and that cable routing is free of interference. Published
weights are 110 g for the adaptive gripper and 60 g for Camera Flange 2.0,
about 170 g combined, excluding any extra mounting/cable load and the block.
This is below the arm's published 250 g payload by mass alone; it neither
establishes overload nor rules out tool moment, cable drag or joint problems.

Sources:
- https://docs.elephantrobotics.com/docs/mycobot_280_pi_en/4-SupportAndService/Accessories/AdaptiveGripper.html
- https://www.elephantrobotics.com/en/mycobot-camera-flange-en/
- https://docs.elephantrobotics.com/docs/mycobot_280_pi_en/1-ProductInformation/2.ProductParameter/2-ProductParameters.html


## Manufacturer-specific procedure reviewed

The 280 Pi hardware troubleshooting page links a PDF titled "Method of
performing pid adjustment for robotic arm". Its 280 Pi example points to
`280_320_pid.rar`, containing `PI_280_320_pid.rar` and
`pi320_280_Change_Joint_Config_Date_V20230603.py`. The archive was downloaded
and inspected as text only; no configuration script was executed.

The Pi script applies this array to every joint J1-J6:

| Address | Script value | Current J2 value | Comparison |
| --- | --- | --- | --- |
| 13 | 90 | Not captured | Unknown |
| 14 | 130 | Not captured | Unknown |
| 21 | 32 | 32 | Matches |
| 22 | 8 | 8 | Matches |
| 23 | 0 | 0 | Matches |
| 24 | 0 | 0 | Matches |
| 26 | 1 | 3 | Differs |
| 27 | 1 | 3 | Differs |

This is a documented profile difference, not confirmation that 26/27 cause
the observed error. The source does not define their units or confirm its
2023 profile applies to this unit with Atom 7.3 and pymycobot 4.0.6. It imports
legacy `MyCobot`, uses baud 115200, and changes all six joints, including
settings on J1/J4/J5/J6 that currently differ. Do not run it unmodified or
assume its values are confirmed factory defaults. Exact register meanings
and authorization for a J2-only change should be obtained from the manufacturer.
The newer SDK demo instead uses 3 for 26/27, so the two published profiles
should be reconciled rather than selecting one arbitrarily.

The current machine is already in refresh mode 1, satisfying that published
troubleshooting suggestion. No exact acceptable J2 steady-state angular error
or firmware-7.3-specific tuning values were found in the reviewed material.
A support-ready request is saved in `j2_support_request.txt`.

Additional primary sources:
- https://drive.google.com/file/d/1UWhaaSTuwLFImuEGY1J2tvgxTQDwWxK_/view
- https://drive.google.com/drive/folders/1FMoIkyENMHMyD5rks_zZQu122f60-F5O
- https://drive.google.com/file/d/1Ofs7VF5icHNh-skSug724blhX2zFefrl/view
- https://www.elephantrobotics.com/en/contact-us-en/
