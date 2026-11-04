# Robotics Core

`robot/software/core/` contains ROS-independent numerical models, control foundations, reusable visualization and interactive engineering work for DUK.

## Implemented foundation

The core now models the established planar leg/wheel geometry rather than a generic serial-chain placeholder.

| Area | Current scope |
| --- | --- |
| `src/duk_core/modeling/leg_linkage.py` | Closed four-bar kinematics, assembly branches, H/O wheel-center construction, geometric workspace, analytic Jacobian, point trajectories and frame transforms |
| `src/duk_core/modeling/mass_properties.py` | Mass-weighted center of mass, parallel-axis pitch inertia and an equivalent-pendulum reduction for later balance modeling |
| `src/duk_core/visualization/leg_linkage.py` | Optional Matplotlib drawing of the mechanism, point trajectories, coordinate curves and Jacobian curves |
| `examples/duk_leg_kinematics.py` | Numerical inspection of the current DUK leg geometry; plotting is optional |
| `tests/` | Geometry invariants, both four-bar branches, workspace limits, analytic Jacobian verification and mass-property tests |
| `notebooks/` | Interactive investigations that call the reusable core API rather than reimplementing equations |

The physical geometry values are stored in [`robot/config/physical/duk_leg_geometry.json`](../../config/physical/duk_leg_geometry.json). The configuration records AB = 131 mm, BC = 42 mm, CD = 135 mm, AD = 81 mm, BH = 119 mm, OH = 55 mm, wheel radius 75 mm and AD sketch orientation 30 degrees.

`assembly_branch` is intentionally unset in the physical configuration. The link lengths admit two circle-intersection branches; the branch used by the actual CAD assembly must be verified before a branch is treated as physical truth. The example therefore selects a branch explicitly for calculation.

## Geometry convention

For loop closure, the model uses a local A-D frame with `A=(0,0)` and `D=(AD,0)`. The input angle is measured from A->D to A->B. After B and C are solved, H and O are constructed from the established local B-xy geometry:

- H, B and C are collinear in the order H-B-C.
- BH is 119 mm opposite B->C.
- OH is 55 mm perpendicular to BC.
- local +y points opposite O, represented by `wheel_side=-1` relative to the left normal of B->C.

The configured AD orientation is a frame transform; it is not added to the linkage input angle during loop closure.

## Run

Install and run the numerical example:

```bash
python -m pip install -e './robot/software/core[test]'
python robot/software/core/examples/duk_leg_kinematics.py --angle-deg 75 --branch 1
python -m pytest -q robot/software/core/tests
```

For plots and notebooks:

```bash
python -m pip install -e './robot/software/core[research]'
python robot/software/core/examples/duk_leg_kinematics.py --angle-deg 75 --branch 1 --plot
```

The model deliberately does **not** invent DUK masses, inertias, actuator limits or controller gains. `mass_properties.py` accepts future CAD-derived or measured values so the same kinematic model can feed the equivalent-pendulum, LQR and whole-body-control work without duplicating geometry equations.

[Core documentation](../../../docs/software/core.md) · [Repository layout](../../../docs/architecture/repository-layout.md).
