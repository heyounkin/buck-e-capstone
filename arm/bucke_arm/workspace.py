"""Plot an offline workspace sampled from Buck-E's joint ranges."""

import argparse
import math
from pathlib import Path
import random

from .arm import SPEC_LIMITS
from .kinematics import forward_kinematics, gripper_midpoint_mat


def boundary_points(points):
    """Keep the farthest sample in each 10-degree direction bin, plus XYZ extrema."""
    axes = tuple(zip(*points))
    center_x, center_y, center_z = ((min(axis) + max(axis)) / 2 for axis in axes)
    farthest = {}
    for point in points:
        x, y, z = point
        dx, dy, dz = x - center_x, y - center_y, z - center_z
        azimuth = math.degrees(math.atan2(dy, dx))
        elevation = math.degrees(math.atan2(dz, math.hypot(dx, dy)))
        direction = (int((azimuth + 180) / 10) % 36,
                     min(17, int((elevation + 90) / 10)))
        distance_squared = dx * dx + dy * dy + dz * dz
        if direction not in farthest or distance_squared > farthest[direction][0]:
            farthest[direction] = distance_squared, point

    selected = [point for _, point in farthest.values()]
    # Preserve the exact sampled limits even when another point in the same
    # direction is farther from the center.
    for axis in axes:
        selected.extend((points[axis.index(min(axis))], points[axis.index(max(axis))]))
    return list(dict.fromkeys(selected))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=30_000,
                        help="Number of random joint configurations (default: 30000)")
    parser.add_argument("--seed", type=int, default=42,
                        help="Random seed for a repeatable plot (default: 42)")
    parser.add_argument("--point", choices=("flange", "gripper"), default="flange",
                        help="Plot the wrist flange or estimated closed-pad midpoint")
    parser.add_argument("--units", choices=("mm", "in"), default="mm",
                        help="Display coordinates in millimetres (default) or inches")
    parser.add_argument("--boundary-only", action="store_true",
                        help="Show only outermost samples around the point cloud")
    parser.add_argument("--output", type=Path, metavar="IMAGE",
                        help="Save to an image (for example workspace.png) instead of opening a window")
    args = parser.parse_args(argv)
    if args.samples < 1:
        parser.error("--samples must be greater than zero")

    try:
        import matplotlib
        if args.output is not None:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        parser.exit(1, "Workspace plotting requires Matplotlib. Install it with "
                       "python -m pip install matplotlib.\n")

    if args.point == "flange":
        def position(angles):
            return forward_kinematics(angles).xyz_mm

        title = "Buck-E wrist flange workspace"
        frame = "Base"
    else:
        position = gripper_midpoint_mat
        title = "Buck-E closed-pad midpoint workspace (provisional)"
        frame = "Mat"

    # Sample joint angles independently. The result shows the modeled range
    # of endpoint positions, without filtering for collisions or clearance.
    rng = random.Random(args.seed)
    points = []
    for _ in range(args.samples):
        angles = [rng.uniform(low, high) for low, high in SPEC_LIMITS]
        points.append(position(angles))
    if args.boundary_only:
        points = boundary_points(points)
        title += "\nOutermost sampled points"
    x, y, z = zip(*points)
    if args.units == "in":
        x, y, z = (tuple(value / 25.4 for value in axis) for axis in (x, y, z))

    figure, axes = plt.subplots(figsize=(9, 7), layout="constrained",
                                subplot_kw={"projection": "3d"})
    cloud = axes.scatter(x, y, z, c=z, cmap="viridis",
                         s=8 if args.boundary_only else 2,
                         alpha=0.85 if args.boundary_only else 0.55, depthshade=False)
    z_label = f"{frame} Z ({args.units})"
    axes.set(title=title, xlabel=f"{frame} X ({args.units})",
             ylabel=f"{frame} Y ({args.units})", zlabel=z_label)
    # Match the drawn box proportions to its data ranges so that one unit
    # has the same scale on every axis before the 3D view is projected.
    axes.set_box_aspect([high - low for low, high in
                         (axes.get_xlim(), axes.get_ylim(), axes.get_zlim())])
    figure.colorbar(cloud, ax=axes, shrink=0.65, pad=0.1, label=z_label)

    try:
        if args.output is None:
            plt.show()
        else:
            figure.savefig(args.output, dpi=180)
            print(f"Saved {len(points):,} of {args.samples:,} workspace samples "
                  f"to {args.output.resolve()}")
    except (OSError, ValueError) as error:
        parser.exit(1, f"Workspace plot failed: {error}\n")
    finally:
        plt.close(figure)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
