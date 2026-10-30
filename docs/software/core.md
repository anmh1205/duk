# Robotics Core

## Purpose and boundary

`robot/software/core/` is the reusable, ROS-independent implementation of robot models, numerical algorithms and scientific visualization helpers. Core is a Python package, `duk_core`, with a conventional `src/` layout and independently executable tests.

Core does not depend on ROS message types, node lifecycle, hardware I/O, Docker runtime paths, specific dashboards or a fixed file location for robot configuration. Dependencies used only for plotting or notebooks remain optional.

## Internal organization

| Area | Contract |
| --- | --- |
| `src/duk_core/modeling/` | Kinematics, dynamics, contact assumptions, state descriptions and trajectories |
| `src/duk_core/control/` | Control-design algorithms, controller evaluation and reusable numerical operations |
| `src/duk_core/visualization/` | Optional reusable geometry drawing, plotting and animation |
| `notebooks/` | Interactive inspection of API values, motion, charts and algorithm behavior |
| `examples/` | Focused examples illustrating supported public APIs |
| `tests/` | Unit tests, numerical invariants and reference cases |

Notebooks and example scripts are consumers of the package, not alternative locations for canonical equations. Place reusable calculations into the source package and give them stable interfaces.

## Modeling contracts

Specify generalized coordinates, robot-state ordering, frame transforms, units, reference configuration and model fidelity. Inputs include physical parameters through well-defined APIs; they must not be read from hard-coded relative paths. Document whether a model is planar, spatial, symbolic, numerical, linearized or reduced-order.

Dynamics and contact models require explicit assumptions about constraints, rolling contact, friction, external forces and actuation. A result validated on a simplified example must not be labeled a validated model of DUK.

## Control contracts

Separate controller design/synthesis (e.g. gain selection, operating-point linearization) from runtime command computation. Document the operating envelope, sampling assumptions, saturation, required state estimate and fallback behavior. Compare control and estimator representations to shared data contracts.

## Visualization and exploration

Matplotlib or other visualization libraries may be used as optional dependencies. Shared plotting primitives operate on numerical results supplied by the caller. Notebook-specific figures can remain local; only generalized and maintained helpers belong in `visualization/`.

## Testing and consumption

Unit tests check shapes, units, numerical results and boundary cases; integration tests verify core behavior when connected to ROS adapters or simulation. ROS consumers install the package and import its API rather than copying the source tree.

See [Data contracts](../architecture/data-contracts.md) and [Testing](../development/testing.md).
