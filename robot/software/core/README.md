# DUK Robotics Core

`duk-core` provides the ROS-independent numerical foundation for developing and evaluating DUK models and control methods. The package uses a conventional Python `src/` layout, with tests, executable examples and notebooks that access the same public APIs.

## Package boundaries

| Location | Responsibility |
| --- | --- |
| `src/duk_core/modeling/` | Kinematics, dynamics, contact constraints, trajectories and state definitions |
| `src/duk_core/control/` | Control design, optimization and numerical controller operations |
| `src/duk_core/visualization/` | Optional plotting, geometry and animation utilities |
| `notebooks/` | Interactive numerical exploration and motion visualization |
| `examples/` | Concise examples of public package APIs |
| `tests/` | Reproducible unit and numerical regression tests |

The modeling and control modules must not require ROS, middleware messages, hardware drivers or graphical dependencies. Visualization dependencies may be installed when needed for notebook work.

## Setup and verification

From the repository root:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e "./robot/software/core[test,research]"
python -m pytest robot/software/core/tests
```

Run JupyterLab from the configured Python environment to work with notebooks under `notebooks/`. `examples/` provides smaller executable usage references.

## Data and model contracts

All algorithm APIs must specify units, coordinate frames, parameter ordering and validity ranges. Physical robot values come from reviewed input configurations, not hard-coded locations within the source package. Any derivation, linearization or reduced-order approximation should document its assumptions and verification method.

The existing generic planar-chain helper illustrates the package interface and drawing workflow; its geometry is not a validated model of DUK. Robot-specific kinematics and dynamics require source mechanical geometry, reference configurations and numerical tests before being treated as authoritative.

## Relationship to the wider repository

ROS integrations consume `duk_core` instead of embedding a duplicate model. Research experiments import the same source implementations and record the code/configuration revision for reproducibility. See [Core architecture](../../../docs/software/core.md), [Data contracts](../../../docs/architecture/data-contracts.md) and [Testing](../../../docs/development/testing.md).
