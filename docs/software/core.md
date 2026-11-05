# Robotics Core

> **Status:** Implemented foundation; dynamic and control layers remain under development.

The robotics core is the ROS-independent source of reusable numerical models and control mathematics for DUK. Equations that describe the physical robot belong here when they are reusable by simulation, notebooks, controller design and tests. Interactive notebooks and visualizations consume these APIs; they must not become alternate implementations of the model.

## Implemented model boundary

The current implementation covers the planar closed four-bar leg geometry and the wheel-center offset defined by BH/OH. It provides loop closure, explicit assembly-branch selection, workspace limits, singularity detection, analytic point Jacobians, trajectory sampling and coordinate-frame transforms. The solver uses a local A-D frame, while the configured robot/sketch frame rotates results so AD is +30 degrees counter-clockwise from +Ox; visualization and notebooks use that standing frame by default. It also provides generic planar mass-property composition so future CAD-derived masses/inertias can be converted into a center of mass and equivalent-pendulum representation without inventing values.

Physical geometry values are owned by `robot/config/physical/duk_leg_geometry.json`. The core does not embed unresolved assembly-branch choices, masses, inertias, actuator limits, controller gains or contact parameters.

## Model flow

```text
physical geometry configuration
        |
        v
closed four-bar kinematics ----> local A-D pose/Jacobian
                                      |
                                      v
                           robot/sketch transform
                         (AD = +30 deg from +Ox)
                                      |
                                      v
                         trajectories / visualization
        |
        v
CAD/measured mass properties
        |
        v
combined CoM + pitch inertia
        |
        v
equivalent balance model (next)
        |
        v
controller design / whole-body coordination (next)
```

## Source locations

- `robot/software/core/src/duk_core/modeling/leg_linkage.py`
- `robot/software/core/src/duk_core/modeling/mass_properties.py`
- `robot/software/core/src/duk_core/visualization/leg_linkage.py`
- `robot/software/core/examples/duk_leg_kinematics.py`
- `robot/software/core/tests/`

The next modeling layer should derive and validate the configuration-dependent balance dynamics from these same geometry and mass-property primitives rather than introducing a second independent robot model.

[Robotics core README](../../robot/software/core/README.md) · [Repository layout](../architecture/repository-layout.md).
