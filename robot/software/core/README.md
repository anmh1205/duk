# Robotics Core

`robot/software/core/` contains ROS-independent numerical models, control foundations, reusable visualization and interactive engineering work for DUK.

## Implemented foundation

The core models the established planar leg/wheel geometry rather than a generic serial-chain approximation.

| Area | Current scope |
| --- | --- |
| `src/duk_core/modeling/leg_linkage.py` | Closed four-bar kinematics, assembly branches, H/O wheel-center construction, geometric workspace, analytic Jacobian, point trajectories and frame transforms |
| `src/duk_core/modeling/mass_properties.py` | Mass-weighted center of mass, parallel-axis pitch inertia and an equivalent-pendulum reduction for later balance modeling |
| `src/duk_core/visualization/leg_linkage.py` | Optional Matplotlib drawing of the mechanism, point trajectories, coordinate curves and Jacobian curves |
| `examples/duk_leg_kinematics.py` | Numerical inspection of the current DUK leg geometry; plotting is optional |
| `tests/` | Geometry invariants, both four-bar branches, workspace limits, analytic Jacobian verification and robot-frame convention tests |
| `notebooks/` | Interactive investigations that call the reusable core API rather than reimplementing equations |

The physical geometry source is [`robot/config/physical/duk_leg_geometry.json`](../../config/physical/duk_leg_geometry.json). It records AB = 131 mm, BC = 42 mm, CD = 135 mm, AD = 81 mm, BH = 119 mm, OH = 55 mm and wheel radius 75 mm.

`assembly_branch = -1` is now the physical branch defined by the CAD reference. The `+1` branch remains mathematically valid but represents the mirror assembly and must not be used as the default robot geometry.

## Coordinate-frame convention

Two frames have distinct responsibilities.

### Solver frame

The four-bar loop is solved in a local numerical frame:

- A = (0, 0);
- D = (AD, 0);
- local +x is the directed vector A→D;
- positive rotation is counter-clockwise;
- the input angle is measured from A→D to A→B.

This frame exists to keep the loop-closure equations simple. It does not define how the mechanism is mounted in the robot.

### Robot frame

Robot-facing coordinates use one explicit directed-vector convention:

- +Ox points right;
- +Oy points up;
- positive rotation is counter-clockwise;
- A is used as the current planar mechanism origin;
- the directed vector **D→A has angle -30° from +Ox**.

Because A→D is the opposite directed vector,

[
\theta_{AD}=\theta_{DA}+180^\circ=-30^\circ+180^\circ=150^\circ.
]

Therefore the solver-to-robot rotation is **+150°**, not +30°. This gives D a greater y-coordinate than A, matching the physical statement that D is higher than A.

The configuration stores the primary physical statement `da_angle_deg = -30`. Code derives the A→D angle from it; the derived 150° value is not duplicated in configuration.

The wheel-center construction remains:

- H, B and C are collinear in the order H-B-C;
- BH = 119 mm opposite B→C;
- OH = 55 mm perpendicular to BC;
- `wheel_side=-1` selects the established side of directed BC.

## Run

Inside the DUK development container:

```bash
python robot/software/core/examples/duk_leg_kinematics.py --angle-deg 75
python -m pytest -q robot/software/core/tests
```

Robot frame and the physical `-1` assembly branch are the defaults. The example angle is only a demo input and is not claimed to reproduce the exact CAD screenshot pose. For solver-level debugging only:

```bash
python robot/software/core/examples/duk_leg_kinematics.py \
  --angle-deg 75 \
  --frame solver
```

For Matplotlib windows:

```bash
python robot/software/core/examples/duk_leg_kinematics.py \
  --angle-deg 75 \
  --plot
```

The model deliberately does **not** invent DUK masses, inertias, actuator limits or controller gains. `mass_properties.py` accepts future CAD-derived or measured values so the same kinematic model can feed balance dynamics and controller design without duplicating geometry equations.

[Core documentation](../../../docs/software/core.md) · [Repository layout](../../../docs/architecture/repository-layout.md).
