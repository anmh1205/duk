"""Inspect the DUK leg directly in canonical robot coordinates."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

from duk_core.modeling import (
    leg_jacobian,
    load_leg_geometry_json,
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
    parser.add_argument(
        "--angle-deg",
        type=float,
        default=75.0,
        help="absolute A->B angle measured counter-clockwise from robot +Ox",
    )
    parser.add_argument("--plot", action="store_true")
    args = parser.parse_args()

    geometry = load_leg_geometry_json(
        repository_geometry_path()
    )
    pose = solve_leg_pose(
        geometry,
        math.radians(args.angle_deg),
    )
    jacobian = leg_jacobian(geometry, pose)
    lower, upper = geometry.geometric_ab_angle_limits_rad()

    print("DUK leg geometry")
    print("  frame: canonical robot frame")
    print("  A is the only fixed coordinate: (0, 0) mm")
    print(
        "  DA direction: "
        f"{math.degrees(geometry.da_angle_rad):+.3f} deg from +Ox"
    )
    print(
        "  AD direction: "
        f"{math.degrees(geometry.ad_angle_rad):+.3f} deg from +Ox"
    )
    print(
        "  physical A->B angle range: "
        f"{math.degrees(lower):.3f} .. "
        f"{math.degrees(upper):.3f} deg "
        "(singular boundaries, not operating limits)"
    )
    print(f"  BO derived: {geometry.bo_mm:.3f} mm")
    print(f"  CO derived: {geometry.co_mm:.3f} mm")
    print(
        "  topology: "
        f"B is {geometry.b_side_of_ad} of A->D, "
        f"C is {geometry.c_side_of_bd} of B->D, "
        f"O is {geometry.bco_body.o_side_of_bc} of B->C"
    )
    print(f"  A->B angle: {args.angle_deg:.3f} deg")
    for name in ("A", "B", "C", "D", "H", "O"):
        x, y = pose.point(name)
        print(f"  {name}: ({x:.3f}, {y:.3f}) mm")
    print(
        "  dO/d(A->B angle): "
        f"({jacobian.d_o[0]:.3f}, "
        f"{jacobian.d_o[1]:.3f}) mm/rad"
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
        )

        draw_leg_linkage(
            pose,
            geometry,
        ).set_title(
            f"DUK leg linkage — A→B={args.angle_deg:.1f}°"
        )
        draw_point_trajectories(
            poses
        ).set_title(
            "DUK linkage point trajectories — canonical robot frame"
        )
        plot_point_coordinates(
            poses,
            "O",
        ).set_title(
            "Wheel-center coordinates — canonical robot frame"
        )
        plot_point_jacobian(
            geometry,
            poses,
            "O",
        ).set_title(
            "Wheel-center Jacobian — canonical robot frame"
        )
        plt.show()


if __name__ == "__main__":
    main()
