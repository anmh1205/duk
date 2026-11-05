# Robotics Core

> **Status:** Implemented foundation; dynamic and control layers remain under development.

The robotics core is the ROS-independent source of reusable numerical models and control mathematics for DUK. Equations describing the physical robot belong here when they are shared by simulation, notebooks, controller design and tests. Interactive notebooks and visualizations consume these APIs; they must not become alternate implementations of the robot model.

## Implemented model boundary

The current implementation covers the closed planar four-bar geometry and the wheel-center offset defined by BH/OH. It provides loop closure, explicit assembly-branch selection, workspace limits, singularity detection, analytic point Jacobians, trajectory sampling and rigid frame transforms.

The implementation deliberately separates the **solver frame** from the **robot frame**.

In the solver frame, A=(0,0) and A→D is aligned with local +x. In the robot frame, +Ox points right, +Oy points up and positive rotation is counter-clockwise. The physical orientation is defined by the directed vector D→A:

[
\theta_{DA}=-30^\circ.
]

The opposite directed vector is therefore:

[
\theta_{AD}=150^\circ.
]

The solver-to-robot transform is consequently a +150° rotation. Pose coordinates, trajectory plots and robot-frame Jacobians all use this same transform. No separate 30°, 90° or presentation-only correction is inserted elsewhere.

Physical geometry values and frame convention are owned by `robot/config/physical/duk_leg_geometry.json`. The core does not embed unresolved assembly-branch choices, masses, inertias, actuator limits, controller gains or contact parameters.

## Model flow

```text
physical geometry + directed DA convention
                |
                v
      solver frame: A->D = +x
                |
                v
      closed four-bar kinematics
         pose + analytic Jacobian
                |
                v
  derive robot A->D direction from D->A
       DA = -30 deg -> AD = +150 deg
                |
                v
      solver-to-robot rigid rotation
                |
                v
   robot-frame pose / trajectories / Jacobian
                |
                v
       CAD/measured mass properties
                |
                v
         combined CoM + inertia
                |
                v
      equivalent balance model (next)
                |
                v
   controller design / coordination (next)
```

## Source locations

- `robot/config/physical/duk_leg_geometry.json`
- `robot/software/core/src/duk_core/modeling/leg_linkage.py`
- `robot/software/core/src/duk_core/modeling/mass_properties.py`
- `robot/software/core/src/duk_core/visualization/leg_linkage.py`
- `robot/software/core/examples/duk_leg_kinematics.py`
- `robot/software/core/notebooks/kinematics/duk-leg-kinematics.ipynb`
- `robot/software/core/tests/`

The next modeling layer should derive configuration-dependent balance dynamics from these same geometry and frame primitives rather than introducing a second independent robot model.

[Robotics core README](../../robot/software/core/README.md) · [Repository layout](../architecture/repository-layout.md).
