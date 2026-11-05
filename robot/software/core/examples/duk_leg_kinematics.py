"""Inspect the established DUK leg geometry without requiring ROS or NumPy."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

from duk_core.modeling import (
    jacobian_in_sketch_frame,
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
    parser.add_argument(
        "--frame",
        choices=("robot", "local"),
        default="robot",
        help="display/output frame; robot keeps AD at configured +30 deg from +Ox",
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
    robot_pose = pose_in_sketch_frame(geometry, pose)
    robot_jacobian = jacobian_in_sketch_frame(geometry, jacobian)
    if args.frame == "robot":
        display_pose = robot_pose
        display_jacobian = robot_jacobian
        frame_label = "robot/sketch frame (AD = +30 deg from +Ox)"
    else:
        display_pose = pose
        display_jacobian = jacobian
        frame_label = "local AD solver frame"
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
    print(f"  display frame: {frame_label}")
    print(
        "  configured AD orientation: "
        f"{math.degrees(geometry.ad_orientation_rad):.3f} deg from +Ox"
    )
    for name in ("A", "B", "C", "D", "H", "O"):
        x, y = display_pose.point(name)
        print(f"  {name}: ({x:.3f}, {y:.3f}) mm")
    print(
        "  dO/dalpha: "
        f"({display_jacobian.d_o[0]:.3f}, "
        f"{display_jacobian.d_o[1]:.3f}) mm/rad"
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
        if args.frame == "robot":
            display_poses = tuple(
                pose_in_sketch_frame(geometry, sampled_pose)
                for sampled_pose in poses
            )
            jacobian_rotation = geometry.ad_orientation_rad
        else:
            display_poses = poses
            jacobian_rotation = 0.0

        draw_leg_linkage(
            display_pose,
            geometry,
        ).set_title(
            f"DUK leg linkage — {frame_label}"
        )
        draw_point_trajectories(
            display_poses
        ).set_title(
            f"DUK linkage point trajectories — {args.frame} frame"
        )
        plot_point_coordinates(
            display_poses,
            "O",
        ).set_title(
            f"Wheel-center coordinates — {args.frame} frame"
        )
        plot_point_jacobian(
            geometry,
            poses,
            "O",
            rotation_rad=jacobian_rotation,
        ).set_title(
            f"Wheel-center Jacobian — {args.frame} frame"
        )
        plt.show()


if __name__ == "__main__":
    main()
