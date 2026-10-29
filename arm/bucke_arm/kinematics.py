"""Offline myCobot 280 Pi flange FK; standard library only, no SDK connection.

Geometry: elephantrobotics/mycobot_ros, revision
c0d6fbe8f7d9b18b46a9fcb0b041e5bc2d66196b,
mycobot_description/urdf/mycobot_280_pi/mycobot_280_pi.urdf.
URDF origins below are converted from metres to mm; RPY stays in radians.
Each local transform is Translation(xyz) Rz(yaw) Ry(pitch) Rx(roll) Rz(q).
SDK J1-J6 degrees map in order to the six revolute joints, including the
URDF joint named joint6output_to_joint6. This mapping is preliminary.
"""

from dataclasses import dataclass
import math


MODEL_REVISION = "c0d6fbe8f7d9b18b46a9fcb0b041e5bc2d66196b"
MODEL_URL = ("https://github.com/elephantrobotics/mycobot_ros/blob/" + MODEL_REVISION
             + "/mycobot_description/urdf/mycobot_280_pi/mycobot_280_pi.urdf")
# Retain the manufacturer's rounded angles instead of substituting exact pi/2.
JOINT_ORIGINS = (
    ((0, 0, 139.56), (0, 0, 0)),
    ((0, 0, -1), (0, 1.5708, -1.5708)),
    ((-110.4, 0, 0), (0, 0, 0)),
    ((-96, 0, 64.62), (0, 0, -1.5708)),
    ((0, -73.18, -1), (1.5708, -1.5708, 0)),
    ((0, 45.6, 0), (-1.5708, 0, 0)),
)


def _values(values, count):
    if (not isinstance(values, (list, tuple)) or len(values) != count
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in values)):
        raise ValueError("Expected %d finite numeric values" % count)
    return tuple(values)


def _multiply(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(4))
                       for j in range(4)) for i in range(4))


def _transform(xyz, rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                             math.sin(p), math.cos(y), math.sin(y))
    return ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr, xyz[0]),
            (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr, xyz[1]),
            (-sp, cp*sr, cp*cr, xyz[2]),
            (0, 0, 0, 1))


@dataclass(frozen=True)
class FlangePose:
    """Base-to-flange transform; rotation columns are flange axes in base frame."""

    transform_mm: tuple

    @property
    def xyz_mm(self):
        return tuple(self.transform_mm[i][3] for i in range(3))

    @property
    def rotation_matrix(self):
        return tuple(row[:3] for row in self.transform_mm[:3])

    def point_in_base(self, offset_mm):
        """Map a flange-local point into base XYZ; offset rotates with the wrist."""
        offset = _values(offset_mm, 3)
        return tuple(self.xyz_mm[i] + sum(self.rotation_matrix[i][j] * offset[j]
                                         for j in range(3)) for i in range(3))

    @property
    def rpy_deg(self):
        """One Rz(yaw) Ry(pitch) Rx(roll) solution; yaw=0 at gimbal lock.

        This is a documented model convention, not a verified SDK convention.
        Use the rotation matrix when comparing orientations near singularities.
        """
        m = self.rotation_matrix
        horizontal = math.hypot(m[0][0], m[1][0])
        pitch = math.atan2(-m[2][0], horizontal)
        if horizontal > 1e-10:
            roll, yaw = math.atan2(m[2][1], m[2][2]), math.atan2(m[1][0], m[0][0])
        else:
            roll, yaw = math.atan2(-m[1][2], m[1][1]), 0.0
        return tuple(math.degrees(v) for v in (roll, pitch, yaw))

    @property
    def pose_mm_deg(self):
        return self.xyz_mm + self.rpy_deg


def forward_kinematics(angles_deg):
    """Return nominal flange pose for six angles, without enforcing motion limits.

    No gripper offset, mat transform, calibration or collision model is applied.
    """
    angles = _values(angles_deg, 6)
    result = _transform((0, 0, 0), (0, 0, 0))
    for (xyz, rpy), angle in zip(JOINT_ORIGINS, angles):
        result = _multiply(result, _transform(xyz, rpy))
        result = _multiply(result, _transform((0, 0, 0), (0, 0, math.radians(angle))))
    return FlangePose(result)


def compare_xyz(angles_deg, reported_xyz_mm):
    """Predicted minus reported flange XYZ; descriptive error, no pass threshold."""
    reported = _values(reported_xyz_mm, 3)
    predicted = forward_kinematics(angles_deg).xyz_mm
    delta = tuple(p - r for p, r in zip(predicted, reported))
    return {"predicted_xyz_mm": predicted, "reported_xyz_mm": reported,
            "delta_xyz_mm": delta,
            "position_error_mm": math.sqrt(sum(d*d for d in delta))}
