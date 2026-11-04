"""Inspect the established DUK leg geometry without requiring ROS or NumPy."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

from duk_core.modeling import (
    leg_jacobian,
    load_leg_geometry_json,
    pose_in_sketch_frame,
    sample_leg_trajectory,
    solve_leg_pose,
)


def repository_geometry_path() -> Path:
    return (
        Path(__file__).resolve().parents[3]
        / "config"
        / "physical"
        / "duk_leg_geometry.json"
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--angle-deg", type=float, default=75.0)
    parser.add_argument(
        "--branch",
        type=int,
        choices=(-1, 1),
        default=1,
    )
    parser.add_argument("--plot", action="store_true")
    args = parser.parse_args()

    geometry = load_leg_geometry_json(
        repository_geometry_path()
    )
    pose = solve_leg_pose(
        geometry,
        math.radians(args.angle_deg),
        assembly_branch=args.branch,
    )
    jacobian = leg_jacobian(geometry, pose)
    lower, upper = geometry.geometric_angle_limits_rad()

    print("DUK leg geometry")
    print(f"  BO derived: {geometry.bo_mm:.3f} mm")
    print(f"  CO derived: {geometry.co_mm:.3f} mm")
    print(
        "  geometric input-angle range: "
        f"{math.degrees(lower):.3f} .. "
        f"{math.degrees(upper):.3f} deg "
        "(singular boundaries, not operating limits)"
    )
    print(
        "  assembly branch used for this calculation: "
        f"{args.branch:+d}"
    )
    for name in ("A", "B", "C", "D", "H", "O"):
        x, y = pose.point(name)
        print(
            f"  {name}: ({x:.3f}, {y:.3f}) mm "
            "in local AD frame"
        )
    print(
        "  dO/dalpha: "
        f"({jacobian.d_o[0]:.3f}, "
        f"{jacobian.d_o[1]:.3f}) mm/rad"
    )

    sketch_pose = pose_in_sketch_frame(
        geometry,
        pose,
    )
    print(
        "  O in configured sketch frame: "
        f"({sketch_pose.o[0]:.3f}, "
        f"{sketch_pose.o[1]:.3f}) mm"
    )

    if args.plot:
        from duk_core.visualization import (
            draw_leg_linkage,
            draw_point_trajectories,
            plot_point_coordinates,
            plot_point_jacobian,
        )
        import matplotlib.pyplot as plt

        margin = math.radians(2.0)
        poses = sample_leg_trajectory(
            geometry,
            lower + margin,
            upper - margin,
            180,
            assembly_branch=args.branch,
        )
        draw_leg_linkage(
            pose,
            geometry,
        ).set_title(
            "DUK leg linkage — local AD frame"
        )
        draw_point_trajectories(
            poses
        ).set_title(
            "DUK linkage point trajectories"
        )
        plot_point_coordinates(
            poses,
            "O",
        ).set_title(
            "Wheel-center coordinates vs input angle"
        )
        plot_point_jacobian(
            geometry,
            poses,
            "O",
        ).set_title(
            "Wheel-center Jacobian vs input angle"
        )
        plt.show()


if __name__ == "__main__":
    main()
