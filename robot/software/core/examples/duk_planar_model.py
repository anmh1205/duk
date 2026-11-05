"""Inspect symbolic equations and optionally bind measured physical parameters.

No synthetic mass/inertia values are used here. Missing/null values remain
symbols. This example does not drive Gazebo, actuators or a controller.
"""

import argparse
import json
import math
from pathlib import Path

from duk_core.modeling import (
    build_planar_biped_model,
    inverse_wheel_position,
    leg_second_derivative,
    load_leg_geometry_json,
    solve_leg_pose,
)


def main():
    root = next(
        p for p in Path(__file__).resolve().parents if (p / "compose.yaml").exists()
    )
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--geometry",
        type=Path,
        default=root / "robot/config/physical/duk_leg_geometry.json",
    )
    parser.add_argument(
        "--parameters",
        type=Path,
        default=root / "robot/config/physical/duk_planar_parameters.example.json",
    )
    parser.add_argument("--angle-deg", type=float, default=75)
    args = parser.parse_args()
    geometry = load_leg_geometry_json(args.geometry)
    pose = solve_leg_pose(geometry, math.radians(args.angle_deg))
    inverse = inverse_wheel_position(geometry, pose.o)
    second = leg_second_derivative(geometry, pose)
    model = build_planar_biped_model(geometry)
    data = json.loads(args.parameters.read_text())
    if data.get("schema_version") != 1 or data.get("model") != "planar_biped_7dof":
        raise ValueError("Expected planar_biped_7dof parameter schema version 1")
    values = data["parameters"]
    unknown_keys = set(values) - model.parameters.keys()
    if unknown_keys:
        raise ValueError(f"Unknown parameter names: {sorted(unknown_keys)}")
    bindings = model.bind_parameters(
        {name: value for name, value in values.items() if value is not None}
    )
    print("Model: ideal sagittal-plane biped; no contact mode or controller selected")
    print("Coordinates:", tuple(str(q) for q in model.coordinates))
    print("Bodies:", len(model.bodies), "mass matrix:", model.mass_matrix.shape)
    print("Equation: M(q) qdd + h(q,qd) + G(q) + D(q) qd = B(q) u + Jc(q)^T lambda")
    print(
        f"Physical parameters: {len(bindings)} bound, {len(model.parameters) - len(bindings)} still symbolic"
    )
    for name, symbol in model.parameters.items():
        print(f"  {name} = {bindings.get(symbol, 'unknown')}")
    print(f"Geometric snapshot q={args.angle_deg:g} deg; wheel O={pose.o} mm")
    print(
        "Inverse q solutions (deg):",
        [math.degrees(p.linkage_angle_rad) for p in inverse],
    )
    print("d2O/dq2 (mm/rad2):", second.dd_o)


if __name__ == "__main__":
    main()
